// Copyright 2020-2021, N Madsen.
// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2021-2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
//
/*!
 * @file
 * @brief Common implementation for WMR controllers, handling
 * shared behaviour such as communication, configuration reading,
 * IMU integration.
 * @author Jan Schmidt <jan@centricular.com>
 * @author Nis Madsen <nima_zero_one@protonmail.com>
 * @ingroup drv_wmr
 */
#pragma once

#include "os/os_threading.h"
#include "math/m_imu_3dof.h"
#include "math/m_relation_history.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "tracking/t_constellation.h"
#include "constellation/t_constellation_tracker.h"

#include "wmr_controller_protocol.h"
#include "wmr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

struct wmr_controller_base;

/*!
 * A connection for communicating with the controller.
 * The mechanism is implementation specific, so there are
 * two variants for either communicating directly with a
 * controller via bluetooth, and another for talking
 * to a controller through a headset tunnelled mapping.
 *
 * The controller implementation doesn't need to care how
 * the communication is implemented.
 *
 * The HMD-tunnelled version of the connection is reference
 * counted and mutex protected, as both the controller and
 * the HMD need to hold a reference to it to clean up safely.
 * For bluetooth controllers, destruction of the controller
 * xrt_device calls disconnect and destroys the connection
 * object (and bluetooth listener) immediately.
 */
struct wmr_controller_connection
{
	//! The controller this connection is talking to.
	struct wmr_controller_base *wcb;

	bool (*send_bytes)(struct wmr_controller_connection *wcc, const uint8_t *buffer, uint32_t buf_size);
	void (*receive_bytes)(struct wmr_controller_connection *wcc,
	                      uint64_t time_ns,
	                      uint8_t *buffer,
	                      uint32_t buf_size);
	int (*read_sync)(struct wmr_controller_connection *wcc, uint8_t *buffer, uint32_t buf_size, int timeout_ms);

	void (*disconnect)(struct wmr_controller_connection *wcc);
};

static inline bool
wmr_controller_connection_send_bytes(struct wmr_controller_connection *wcc, const uint8_t *buffer, uint32_t buf_size)
{
	assert(wcc->send_bytes != NULL);
	return wcc->send_bytes(wcc, buffer, buf_size);
}

static inline int
wmr_controller_connection_read_sync(struct wmr_controller_connection *wcc,
                                    uint8_t *buffer,
                                    uint32_t buf_size,
                                    int timeout_ms)
{
	return wcc->read_sync(wcc, buffer, buf_size, timeout_ms);
}

static inline void
wmr_controller_connection_disconnect(struct wmr_controller_connection *wcc)
{
	wcc->disconnect(wcc);
}

/*!
 * Common base for all WMR controllers.
 *
 * @ingroup drv_wmr
 * @implements xrt_device
 */
struct wmr_controller_base
{
	//! Base struct.
	struct xrt_device base;

	//! Mutex protects the controller connection
	struct os_mutex conn_lock;

	//! The connection for this controller.
	struct wmr_controller_connection *wcc;

	//! Callback from the connection when a packet has been received.
	void (*receive_bytes)(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t buf_size);

	enum u_logging_level log_level;

	//! Mutex protects shared data used from OpenXR callbacks
	struct os_mutex data_lock;

	//! Callback to parse a controller update packet and update the input / imu info. Called with the
	//  data lock held.
	bool (*handle_input_packet)(struct wmr_controller_base *wcb,
	                            uint64_t time_ns,
	                            uint8_t *buffer,
	                            uint32_t buf_size);

	/* firmware configuration block */
	struct wmr_controller_config config;

	/*!
	 * Real 16-byte firmware serial number, read once at connect by read_controller_fw_info()
	 * (2026-09-05). Kept deliberately SEPARATE from @ref base's `serial` field, which is
	 * hardcoded to the literal "Left/Right Controller" (see the TODOs in
	 * wmr_controller_base_init) -- that hardcoding is left alone here; this is purely an
	 * additional debug-visible field. Empty ("") until read_controller_config() succeeds.
	 */
	char fw_serial[16 + 1];

	/*!
	 * Generic mirror of the idle/imu-zeroed flag (2026-09-05), so code that only knows about
	 * this common base -- not any variant-specific input struct -- can tell "idle" apart from
	 * "disconnected" for the dashboard hmd-status.json snapshot (see wmr_hmd.c). Currently only
	 * ever set by the HP variant (wmr_controller_hp.c, right after computing its own
	 * last_inputs.imu.zeroed); left false (the correct "unknown"/never-idle default) for
	 * variants that don't compute this, such as Odyssey.
	 */
	bool imu_zeroed;

	//! Time of last IMU sample, in CPU time.
	uint64_t last_imu_timestamp_ns;
	//! Monotonic time of the last WMR_CONTROLLER_KEEPALIVE_S resend, or 0 before the first one.
	//! See the keepalive prototype in wmr_controller_base_get_tracked_pose. Diagnostic-only,
	//! off by default.
	uint64_t last_keepalive_ns;
	//! Main fusion calculator.
	struct m_imu_3dof fusion;
	//! The last angular velocity from the IMU, for prediction.
	struct xrt_vec3 last_angular_velocity;

	/*!
	 * Optical LED constellation tracking state (WMR_CONSTELLATION_CONTROLLERS). Only populated
	 * after a successful @ref wmr_controller_base_add_to_constellation_tracker call; otherwise
	 * constellation_device_id stays XRT_CONSTELLATION_INVALID_DEVICE_ID and none of this is
	 * used. Orientation tracking (fusion, above) is entirely unaffected either way.
	 */
	struct
	{
		//! Backing storage for constellation_led_model.leds, converted once from config.leds.
		struct t_constellation_tracker_led leds[WMR_MAX_LEDS];
		struct t_constellation_tracker_led_model led_model;
		struct t_constellation_tracker_device device;
		t_constellation_device_id_t device_id;
		//! NULL unless device_id is valid; used only to remove the device from the tracker on destroy.
		struct t_constellation_tracker *tracker;

		/*!
		 * Latest sample from the tracker, for the debug GUI only -- nothing reads this for the actual
		 * device pose yet. Written from the tracker's own thread in
		 * @ref wmr_controller_base_constellation_sample_store, read by the GUI thread via the
		 * u_var_add_ro_* registrations in @ref wmr_controller_base_add_to_constellation_tracker.
		 * Intentionally unsynchronized, same as the rest of this codebase's GUI-only debug fields
		 * (e.g. wmr_camera.c's exposure/gain widgets) -- a torn read just shows a stale frame's value
		 * for one GUI refresh, never a correctness issue.
		 */
		struct xrt_pose last_pose;
		int64_t last_timestamp_ns;
		struct t_constellation_tracker_sample_metrics last_metrics;
		//! Samples received since registration, so "stuck at 0" is visible at a glance in the GUI.
		uint64_t sample_count;
		//! Samples rejected for placing the controller implausibly far away, see the range check.
		uint64_t out_of_range_count;
		//! Samples rejected by the IMU gravity gate (wrong lobe of the solve's bistability).
		uint64_t gravity_gate_drop_count;
		//! Gate-accepted solves that fed WMR_CONTROLLER_SOLVE_YAW_CORRECT (0 while off).
		uint64_t solve_yaw_correction_count;
		//! Per-device get_tracked_pose call counter for the throttled output log. MUST be
		//! per-device: a function-local static is shared by both controllers, and with the
		//! two hands' calls interleaving evenly, an even modulo lands on the same hand every
		//! time -- silently muting the other (the exact bug the 2026-08-11 note below the
		//! use site describes; it had crept back in via a shared static).
		uint64_t get_tracked_pose_call_count;
		//! Set once the solve-yaw error has been observed small (heading acquired);
		//! afterwards, huge sudden errors are distrusted as 0047's yaw-ghost solves.
		bool solve_yaw_locked;
		//! Gate-accepted solves further rejected by WMR_CONSTELLATION_YAW_PRIOR_DEG for
		//! disagreeing with the locked fusion heading (0 while off, or before lock).
		uint64_t yaw_prior_reject_count;
		//! T223 (2026-08-19, docs/58): reverb-g2's own instrumentation, not upstream. Counts
		//! every gravity-gated sample seen while WMR_CONSTELLATION_GRAVITY_GATE_DEG is on, purely
		//! to throttle the "yaw lock status" heartbeat log -- see its use site in
		//! constellation_sample_store. MUST be per-device, same reasoning as
		//! get_tracked_pose_call_count just above: a shared static would silently mute one hand's
		//! heartbeat under interleaved calls.
		uint64_t yaw_lock_status_log_count;

		/*!
		 * Every constellation sample this controller receives, keyed by the sample's own
		 * timestamp. This is what both the output pose and the tracker's prior are read from,
		 * interpolated/predicted to the timestamp actually being asked about -- the same thing
		 * the rift and pssense drivers do with their own constellation samples.
		 */
		struct m_relation_history *relation_history;

		/*!
		 * Handed to the tracker as this device's prior (@ref
		 * t_constellation_tracker_device_params.tracking_source), so it can throw out pose
		 * hypotheses that disagree with where this controller just was. Reads @ref
		 * relation_history.
		 */
		struct t_constellation_tracker_tracking_source tracking_source;
	} constellation;

	/*!
	 * Per-stick center auto-calibration state (WMR_STICK_AUTOCENTER, default off). WMR
	 * thumbstick configs carry no factory center calibration, so a resting stick can read
	 * up to ~0.3 off zero on either axis -- see @ref wmr_controller_base_apply_stick_autocenter.
	 */
	struct
	{
		//! Set once the sampling window has closed with a frozen @ref offset, which is then
		//! subtracted from every later sample. Mutually exclusive with @ref aborted.
		bool locked;
		//! Set if the window ever saw the stick move past the resting bound, or get clicked,
		//! before it could close -- autocenter gives up for this controller's lifetime and
		//! @ref wmr_controller_base_apply_stick_autocenter passes the stick through unmodified.
		bool aborted;
		//! CPU monotonic time of this controller's first stick sample, 0 before it arrives.
		uint64_t window_start_ns;
		//! Number of samples accumulated into @ref accum so far.
		uint32_t sample_count;
		//! Running sum of raw (pre-deadzone) stick x/y while the window is open.
		struct xrt_vec2 accum;
		//! Frozen per-stick center offset, valid only once @ref locked is true.
		struct xrt_vec2 offset;
	} stick_autocenter;

	/*!
	 * WMR_CONTROLLER_HAPTICS state (default off, UNVALIDATED PROTOTYPE -- see @ref
	 * wmr_controller_base_set_output's own comment for the wire-format caveat).
	 */
	struct
	{
		//! Monotonic time of the last haptic report actually written to the tunnel, 0
		//! before the first one. Throttles @ref wmr_controller_base_set_output so a
		//! per-frame-called app can't flood the shared HID tunnel.
		uint64_t last_send_ns;
		//! Set once the first haptic report has been sent for this device, so later
		//! ones can log at DEBUG instead of INFO.
		bool logged_first;
	} haptics;
};

/*!
 * Builds a constellation LED model from this controller's already-parsed factory calibration
 * (@ref wmr_controller_config.leds) and registers it with @p tracker. No-op if @p tracker is
 * NULL (WMR_CONSTELLATION_CONTROLLERS off or tracker creation failed) or @p wcb has no LEDs.
 *
 * @ingroup drv_wmr
 */
void
wmr_controller_base_add_to_constellation_tracker(struct wmr_controller_base *wcb,
                                                  struct t_constellation_tracker *tracker,
                                                  struct xrt_tracking_origin *head_origin);

bool
wmr_controller_base_init(struct wmr_controller_base *wcb,
                         struct wmr_controller_connection *conn,
                         enum xrt_device_type controller_type,
                         enum u_logging_level log_level,
                         u_device_destroy_function_t destroy_fn);

void
wmr_controller_base_deinit(struct wmr_controller_base *wcb);

/*!
 * Apply the optional radial thumbstick deadzone (WMR_STICK_DEADZONE, default 0 =
 * off) to a raw-scaled stick vector, rescaling so full deflection still reaches
 * magnitude 1. WMR controller configs carry no stick centre calibration, so
 * per-unit centre offset otherwise shows up as constant drift in apps.
 */
void
wmr_controller_base_apply_stick_deadzone(struct xrt_vec2 *stick);

/*!
 * Optional per-stick center auto-calibration (WMR_STICK_AUTOCENTER, default off). WMR
 * controller configs carry no factory stick-center calibration (see @ref
 * wmr_controller_base_apply_stick_deadzone above), so a resting stick can read up to ~0.3
 * off zero on either axis -- the wearer's own symptom was the left stick self-pressing
 * up+left and the right pressing hard-left + slightly up. During the first ~5s (or 500
 * samples, whichever closes the window first) after @p wcb's first stick sample, while the
 * raw magnitude stays under a plausible resting bound and the stick isn't clicked, this
 * accumulates the mean raw position; once the window closes, that mean freezes as this
 * stick's center offset and is subtracted (then clamped to [-1,1]) from every later sample.
 * If the window instead sees the stick move past the resting bound, or get clicked, first
 * (the wearer grabbed it during boot) -- autocenter aborts for this controller's lifetime,
 * logs a WARN, and @p stick passes through unmodified from then on: a wrong center is worse
 * than none. Call this BEFORE @ref wmr_controller_base_apply_stick_deadzone, with the same
 * raw per-packet stick vector, so the deadzone can later shrink now that it's not also
 * covering the factory center offset. Logs one INFO per controller when the center freezes
 * (per-unit health data, like the battery roster).
 *
 * @param wcb      This controller.
 * @param stick    Raw (pre-deadzone) stick vector for this packet, corrected in place once locked.
 * @param clicked  Whether the stick is currently pressed in (click); aborts the window.
 */
void
wmr_controller_base_apply_stick_autocenter(struct wmr_controller_base *wcb, struct xrt_vec2 *stick, bool clicked);

/*!
 * WMR_CONTROLLER_KEEPALIVE_S v2 (UNVALIDATED PROTOTYPE, default off): resend the two
 * connect-time enable commands to this controller if the configured interval has elapsed.
 * No-op, cheap, if the env var is unset. Meant to be called from a context that ticks
 * regardless of connected OpenXR clients -- currently wmr_hmd.c's own read thread, once per
 * loop iteration for each connected controller. Do NOT call this from a connection's
 * receive_bytes callback: see the implementation's comment for why that specific call site
 * deadlocks.
 *
 * @param xdev A device created by wmr_controller_base_init().
 */
void
wmr_controller_base_send_keepalive_if_due(struct xrt_device *xdev);

/*!
 * WMR_CONTROLLER_HAPTICS (UNVALIDATED PROTOTYPE, default off): xrt_device::set_output
 * implementation shared by wmr_controller_hp.c (HP Reverb G2) and wmr_controller_og.c
 * (Odyssey/Odyssey+) -- both already declare a single output at @ref
 * wmr_controller_base::base's outputs[0] (XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC or
 * XRT_OUTPUT_NAME_ODYSSEY_CONTROLLER_HAPTIC / XRT_OUTPUT_NAME_WMR_HAPTIC respectively), so
 * this only needs to compare @p name against that. See the implementation's own comment
 * for the full picture: the OUTPUT NAME resolves and is throttled/rate-limited correctly
 * regardless of the env var, but the actual wire report bytes sent when
 * WMR_CONTROLLER_HAPTICS=1 are a best-candidate GUESS, not a confirmed reverse-engineered
 * format -- see docs/03-controllers.md and docs/09-oasis-driver-re.md in the reverb-g2 repo.
 *
 * @param xdev  A device created by wmr_controller_hp_create() or wmr_controller_og_create().
 */
xrt_result_t
wmr_controller_base_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value);

static inline void
wmr_controller_connection_receive_bytes(struct wmr_controller_connection *wcc,
                                        uint64_t time_ns,
                                        uint8_t *buffer,
                                        uint32_t buf_size)
{

	if (wcc->receive_bytes != NULL) {
		wcc->receive_bytes(wcc, time_ns, buffer, buf_size);
	} else {
		/* Default: deliver directly to the controller instance */
		struct wmr_controller_base *wcb = wcc->wcb;
		assert(wcb->receive_bytes != NULL);
		wcb->receive_bytes(wcb, time_ns, buffer, buf_size);
	}
}

#ifdef __cplusplus
}
#endif
