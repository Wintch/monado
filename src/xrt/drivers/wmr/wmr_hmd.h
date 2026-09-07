// Copyright 2018, Philipp Zabel.
// Copyright 2020-2021, N Madsen.
// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the WMR HMD driver code.
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author nima01 <nima_zero_one@protonmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Nova King <technobaboo@proton.me>
 * @ingroup drv_wmr
 */

#pragma once

#include "tracking/t_tracking.h"
#include "tracking/t_constellation.h"
#include "constellation/t_constellation_tracker.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_prober.h"
#include "os/os_threading.h"
#include "math/m_imu_3dof.h"
#include "util/u_logging.h"
#include "util/u_distortion_mesh.h"
#include "util/u_var.h"

#include "wmr_protocol.h"
#include "wmr_config.h"
#include "wmr_camera.h"
#include "wmr_common.h"
#include "wmr_hmd_controller.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Support 2 controllers on HP Reverb G2 */
#define WMR_MAX_CONTROLLERS 2

struct wmr_hmd;

struct wmr_headset_descriptor
{
	enum wmr_headset_type hmd_type;

	//! String by which we recognise the device
	const char *dev_id_str;
	//! Friendly ID string for debug
	const char *debug_name;

	int (*init_func)(struct wmr_hmd *wh);
	void (*deinit_func)(struct wmr_hmd *wh);
	void (*screen_enable_func)(struct wmr_hmd *wh, bool enable);
	//! Optional. Re-sends the family-specific "wake up the companion channel" handshake
	//! (the 0x50 loop + identification reads init_func already does at cold activation)
	//! without touching hmd_screen_enable state. docs/103 (2026-09-06): proven live that
	//! this, not screen_enable_func alone, is what makes the companion answer with a fresh
	//! proximity reading after auto-standby has blanked it -- screen_enable_func by itself
	//! never did. NULL on families that don't need/have this quirk (e.g. Odyssey+).
	void (*reassert_func)(struct wmr_hmd *wh);
	//! Optional. Same motivation as reassert_func, opposite direction: sends the
	//! family-specific screen-OFF command over a brand-new hidraw fd instead of
	//! wh->hid_control_dev. docs/103 (2026-09-06): auto-standby's blank call was still
	//! using the shared-handle screen_enable_func(wh, false) -- the exact handle already
	//! proven unreliable for restore -- and live-caught leaving the panel lit, with real
	//! video still showing, after "panel blanked by auto-standby" had already logged.
	//! NULL on families that don't need/have this quirk (e.g. Odyssey+), same as
	//! reassert_func.
	void (*screen_off_fresh_fd_func)(struct wmr_hmd *wh);
};

/*!
 * @implements xrt_device
 */
struct wmr_hmd
{
	struct xrt_device base;

	const struct wmr_headset_descriptor *hmd_desc;

	//! firmware configuration block, with device names etc
	struct wmr_config_header config_hdr;

	//! Config data parsed from the firmware JSON
	struct wmr_hmd_config config;

	//! Packet reading thread.
	struct os_thread_helper oth;

	enum u_logging_level log_level;

	/*!
	 * This is the Hololens Sensors device, this is where we get all of the
	 * IMU data and read the config from.
	 *
	 * During start it is owned by the thread creating the device, after
	 * init it is owned by the reading thread. Read/write access is
	 * protected by the hid_lock
	 */

	struct os_hid_device *hid_hololens_sensors_dev;
	struct os_mutex hid_lock;

	/*!
	 * Consecutive os_hid_read() failures on hid_hololens_sensors_dev, reset on
	 * any successful read. Bounds how long a transient hiccup is tolerated
	 * before giving up on the IMU/SLAM/controller-tunnel feed for good.
	 */
	int hololens_consecutive_read_errors;

	/*!
	 * This is the vendor specific companion device of the Hololens Sensors.
	 * When activated, it will report the physical IPD adjustment and proximity
	 * sensor status of the headset. It also allows enabling/disabling the HMD
	 * screen on Reverb G1/G2.
	 */
	struct os_hid_device *hid_control_dev;

	/*!
	 * Consecutive os_hid_read() failures on hid_control_dev, reset on any
	 * successful read. Past a threshold, control_read_packets() backs off with a
	 * short sleep instead of retrying unbounded -- see docs/pruebas.jsonl T188,
	 * where an unrecovering companion dropout pinned monado-service at 400%+ CPU
	 * for the length of a real session, because the outer thread loop's usual
	 * pacing (hid_hololens_sensors_dev's blocking read) stops blocking once real
	 * IMU data is arriving quickly. Unlike hololens_consecutive_read_errors this
	 * never gives up -- the companion device isn't load-bearing for tracking.
	 */
	int companion_consecutive_read_errors;
	/*!
	 * When nonzero, skip companion read attempts until this monotonic timestamp instead
	 * of sleeping in the shared read loop. See control_read_packets: the old in-loop
	 * 10ms sleep throttled the hololens sensors reads and let the IMU stream fall a
	 * fixed ~630ms behind (docs/44, T199).
	 */
	uint64_t companion_backoff_until_ns;

	/*!
	 * Identity of the companion USB device, remembered so the read thread can find its
	 * NEW hidraw node after the device re-enumerates. See wmr_hmd_companion_reconnect().
	 */
	uint16_t companion_vid;
	uint16_t companion_pid;

	/*!
	 * Hot-reconnect state for the companion device (reverb-g2 T227, docs/61).
	 *
	 * The USB2 branch this device lives on re-enumerates constantly -- measured at
	 * 3.47 drops/min with ~3 s outages, and measured at the SAME rate on Windows with
	 * the same cable and machine (docs/60), so it is the link, not this stack. What IS
	 * this stack's is the consequence: a re-enumeration invalidates the open hidraw fd
	 * permanently, every later read returns -1 forever, and everything riding this
	 * channel -- panel control, IPD, the proximity sensor behind XR_EXT_user_presence --
	 * stays dead until the service is relaunched. Windows rides the same outages and the
	 * wearer notices nothing but audio. So: re-open the device instead of retrying a
	 * corpse.
	 *
	 * companion_dead_since_ns is when the current dead stretch began (0 = alive), and is
	 * what makes the recovery measurable rather than merely plausible.
	 */
	uint64_t companion_dead_since_ns;
	//! Do not attempt a re-open before this monotonic time (rate limit, ~1/s).
	uint64_t companion_reconnect_next_ns;
	//! Successful re-opens this session, and re-open attempts that failed with the node present.
	uint32_t companion_reconnect_count;
	uint32_t companion_reconnect_failures;

	//! Current desired HMD screen state.
	bool hmd_screen_enable;
	//! Latest raw IPD value read from the device.
	uint16_t raw_ipd;
	//! Latest proximity sensor value read from the device.
	uint8_t proximity_sensor;
	//! Throttle counter for the raw HMD IMU temperature log (2026-09-04), incremented once
	//! per decoded sensor packet (~250 Hz) in hololens_sensors_decode_packet().
	uint32_t temperature_log_count;

	/*!
	 * Raw bytes of the last WMR_CONTROL_MSG_DEVICE_STATUS (0x05) companion message, stored
	 * and exposed VERBATIM (2026-09-05) -- see the parse switch in
	 * wmr_hmd_read_control_packet(). Most fields are still unconfirmed/undecoded; only kept
	 * here for visibility (debug GUI + throttled log), never interpreted.
	 */
	uint8_t device_status_raw[11];
	//! Hex-formatted copy of @ref device_status_raw for u_var_add_ro_text (which wants a string).
	char device_status_hex[64];
	//! Monotonic timestamp of the last DEVICE_STATUS log line, for the ~1/s throttle. 0 = never
	//! logged yet. Time-based (not a modulo counter like @ref temperature_log_count) because
	//! this message is rare and event-driven, not periodic.
	uint64_t device_status_last_log_ns;

	/*!
	 * Snapshot of each tracking camera's computed calibration (pixel-scaled intrinsics +
	 * distortion + the WMR-specific extra params), taken once at startup in
	 * wmr_hmd_fill_slam_cams_calibration() via @ref wmr_hmd_get_cam_calib -- i.e. exactly what
	 * SLAM uses. Kept here (rather than recomputed) purely so the debug GUI can point at live
	 * fields (2026-09-05). Indexed like config.tcams; only [0, config.tcam_count) are valid.
	 */
	struct t_camera_calibration cam_calib_snapshot[WMR_MAX_CAMERAS];

	/*!
	 * Debounce state for XR_EXT_user_presence (reverb-g2 T223/T224).
	 *
	 * The raw byte is NOT clean: measured live during a real donning gesture it alternated
	 * 0,1,0,1 before settling at 1. Reported straight through, that flicker would toggle
	 * presence, and a presence toggle is what a title turns into a pause -- i.e. the game
	 * pausing and unpausing in a wearer's face while they are still putting the headset on.
	 * So a candidate state must persist before it is committed.
	 *
	 * The two directions are deliberately NOT symmetric, and the asymmetry is the whole
	 * point: entering "worn" is cheap to get wrong (a spurious resume is invisible), while
	 * entering "not worn" is expensive (a spurious pause interrupts a session). The same
	 * reasoning the T224 session recorded for the channel-death case: this feature rides the
	 * companion device, the least reliable channel in the stack, and it must always fail
	 * TOWARD "worn", never toward "absent".
	 *
	 * 2026-09-06 addendum (docs/103): that "cheap to get wrong" assumption turned out to
	 * hide a real bug once auto-standby started acting on `committed`, not just an OpenXR
	 * app's pause state -- a spurious WORN commit doesn't just look invisible, it actively
	 * UNDOES a real auto-standby blank. A single stray packet was enough: this function
	 * runs far faster (~250 Hz, measured -- see WMR_USER_PRESENCE_DON_CONFIRM_PACKETS' own
	 * comment in wmr_hmd.c for the derivation) than the companion's own packets arrive
	 * (irregularly, 2 s to 100+ s apart live-measured), so WMR_USER_PRESENCE_DON_MS's
	 * wall-clock window was being satisfied by one unconfirmed byte, not by anything
	 * actually re-observed. Fixed with WMR_USER_PRESENCE_DON_CONFIRM_PACKETS (see that
	 * option's own comment and the two
	 * candidate_confirm_count* fields below) -- entering "worn" is still cheaper than
	 * leaving it, just no longer cheap enough to be satisfied by noise alone.
	 *
	 * 2026-09-06 addendum #2 (docs/103, same day, live wearer test): the packet-count fix
	 * above traded one bug for another. A genuine, deliberate don's candidate committed to
	 * nothing for 13+ seconds live (and, per the full captured log, would have run past 43 s
	 * before the very next packet CONTRADICTED it instead of confirming it -- the don was
	 * never registered at all), because the companion's packets are simply too sparse to
	 * reliably deliver a 2nd confirming sample within any latency a wearer will tolerate.
	 * Fixed by adding a second, independent corroboration source that does not depend on
	 * the sparse channel at all -- see WMR_USER_PRESENCE_DON_MOTION_RAD_S and
	 * candidate_motion_peak_rad_s below -- plus a bounded last-resort timeout
	 * (WMR_USER_PRESENCE_DON_CONFIRM_TIMEOUT_MS) so a real don can never hang indefinitely
	 * even if neither corroboration source shows up in time. See both options' own comments
	 * in wmr_hmd.c for the full reasoning, including the honest limitation of the timeout
	 * path.
	 *
	 * 2026-09-06 addendum #3 (docs/103, later same night, live-caught): addendum #2's
	 * motion CORROBORATION is worthless when the candidate never exists in the first
	 * place. Live-observed the same night: a wearer donned the headset for 5+ minutes
	 * with the panel auto-standby-blanked, and `raw_proximity`/`candidate` in the
	 * PRESENCE-DIAG heartbeat never moved off their NOT-WORN value for the entire
	 * stretch -- zero WMR_CONTROL_MSG_IPD_VALUE packets arrived, worn or not, the whole
	 * time (confirmed against the actual captured jack-in-wayland.log, not assumed). Since
	 * addendum #2's motion peak/timeout/packet corroboration all live INSIDE
	 * `if (wh->presence.candidate != wh->presence.committed)` -- which only runs once the
	 * raw proximity byte has ALREADY flipped -- a channel that never flips at all means
	 * NONE of that logic, including the "hard" timeout, ever executes. Even repeated
	 * periodic reassert pokes (WMR_PRESENCE_REASSERT_INTERVAL_MS, already firing every 15s
	 * in the observed log) did not restore packet delivery during this stretch -- new
	 * evidence that "reassert wakes the channel" (docs/103, ~12:30) is not reliable enough
	 * to depend on either.
	 *
	 * Fixed by promoting motion from a candidate-corroborator to an independent PRIMARY
	 * signal: `worn_signal` (wmr_hmd_presence_tick(), computed before the debounce block)
	 * is `true` whenever the raw proximity byte says so (unchanged, always trusted first),
	 * OR whenever the channel has gone quiet/never spoken for
	 * WMR_USER_PRESENCE_MOTION_RESCUE_STALE_MS and a SUSTAINED motion run (see
	 * motion_run_started_ns/motion_last_above_ns below, and
	 * WMR_USER_PRESENCE_MOTION_PRIMARY_RAD_S/_MOTION_SUSTAIN_MS/_MOTION_GAP_MS) is
	 * currently in progress. Deliberately one-directional: this can only push worn_signal
	 * toward WORN, never toward NOT-WORN -- see wmr_hmd_presence_tick()'s own comment for
	 * why a "motion went quiet -> commit NOT WORN" rule was considered and rejected (a
	 * genuinely worn-but-still wearer, e.g. watching static content, is common and would
	 * false-doff under that rule; a genuinely NOT-worn, undisturbed desk is not, so the
	 * asymmetry is intentional and mirrors this struct's existing "fail toward worn" logic
	 * for the DON/DOFF windows above).
	 */
	struct
	{
		//! State currently reported to the app.
		bool committed;
		//! Candidate state waiting out its debounce window, and when it first appeared.
		bool candidate;
		uint64_t candidate_since_ns;
		//! WMR_USER_PRESENCE_DON_CONFIRM_PACKETS (2026-09-06, docs/103 addendum): number of
		//! independent proximity packets that have reported the CURRENT candidate value,
		//! reset to 1 whenever the candidate flips (the packet that caused the flip is the
		//! first confirmation) and incremented only when a genuinely NEW packet re-confirms
		//! the same value -- see candidate_last_counted_update_ns just below for how "new"
		//! is detected. This exists because wmr_hmd_presence_tick() runs at the read
		//! thread's full ~250 Hz rate (corrected 2026-09-06 from an earlier ~750 Hz guess --
		//! see WMR_USER_PRESENCE_DON_CONFIRM_PACKETS' own comment in wmr_hmd.c for the real,
		//! measured derivation) while the companion's own packets arrive irregularly (2 s to
		//! 100+ s apart, live-measured): without this counter, a single stray packet
		//! satisfies WMR_USER_PRESENCE_DON_MS's wall-clock window all by itself, because
		//! every tick in between just re-reads the same static, unconfirmed byte. Only
		//! consulted for the WORN direction -- see the debug option's own comment for why
		//! NOT WORN is left alone.
		uint64_t candidate_confirm_count;
		//! The presence.last_update_ns value already counted into candidate_confirm_count.
		//! A tick where wh->presence.last_update_ns still equals this is just this
		//! function re-running on the same already-counted packet (or no packet ever
		//! arrived yet) -- NOT a new confirmation. Only when last_update_ns has advanced
		//! past this did a genuinely new packet arrive since the last count, which is what
		//! actually re-confirms the candidate value independently.
		uint64_t candidate_last_counted_update_ns;
		//! WMR_USER_PRESENCE_DON_MOTION_RAD_S (2026-09-06, docs/103 "committed stayed 0 for
		//! 13+s" addendum): peak |wh->fusion.last_angular_velocity| (rad/s) observed on any
		//! tick since the candidate value last changed. Reset (to this tick's own magnitude,
		//! not 0 -- the tick that flips the candidate may already carry real motion) whenever
		//! the candidate flips, updated every tick thereafter while the candidate is WORN --
		//! see wmr_hmd_presence_tick()'s own comment for why this exists: the companion's
		//! proximity packets are too sparse to corroborate a real don within a wearable
		//! latency (this exact log showed a genuine don's candidate held 43+ s, then got
		//! contradicted by the next packet before ever committing), while the IMU updates
		//! every tick regardless, giving a MUCH more available real-time corroboration
		//! signal for "is something physically handling this thing right now" than waiting
		//! on the next rare proximity sample. Only consulted for the WORN direction, same as
		//! candidate_confirm_count above.
		float candidate_motion_peak_rad_s;
		//! WMR_USER_PRESENCE_MOTION_SUSTAIN_MS/_MOTION_GAP_MS (2026-09-06, docs/103
		//! addendum #3): 0 if no motion "run" is currently in progress, otherwise the
		//! monotonic time the CURRENT run began -- i.e. the tick where
		//! |wh->fusion.last_angular_velocity| first rose above
		//! WMR_USER_PRESENCE_MOTION_PRIMARY_RAD_S after being below it (or at startup).
		//! A run ends (reset to 0) once too long a gap (_MOTION_GAP_MS) passes without a
		//! tick back above threshold -- see motion_last_above_ns just below. Tracked on
		//! EVERY tick, unconditionally, independent of candidate/committed/raw proximity --
		//! this has to reflect real-world physical motion regardless of what the proximity
		//! channel is currently saying (or not saying), because it exists specifically to
		//! cover the case where that channel has nothing to say at all. Used to compute
		//! worn_signal in wmr_hmd_presence_tick() -- see that function's own comment.
		uint64_t motion_run_started_ns;
		//! The most recent tick's timestamp where |angular velocity| was at or above
		//! WMR_USER_PRESENCE_MOTION_PRIMARY_RAD_S, i.e. the end of the current motion run
		//! so far. Compared against "now" every tick to detect a run ending (a gap longer
		//! than WMR_USER_PRESENCE_MOTION_GAP_MS since this timestamp resets
		//! motion_run_started_ns to 0) -- this tolerance exists so a single noisy tick
		//! dipping momentarily below threshold mid-gesture doesn't spuriously reset an
		//! otherwise-continuous real donning motion.
		uint64_t motion_last_above_ns;
		//! When the companion last delivered a proximity value at all. 0 = never.
		uint64_t last_update_ns;
		//! Throttle for the stale-channel notice.
		uint64_t stale_log_count;
		//! Monotonic time the current NOT-WORN stretch began (0 = worn, or not-worn but
		//! not yet timed since the last screen-state change). Feeds
		//! WMR_USER_PRESENCE_SCREENOFF_MS in wmr_hmd_update_inputs() -- see there.
		uint64_t not_worn_since_ns;
		//! True once this NOT-WORN stretch has already blanked the panel via
		//! screen_enable_func, so update_inputs doesn't re-call it every frame.
		bool screen_off_by_presence;
		//! Monotonic time of the last reassert_func call made while blanked (0 = none yet
		//! this blanked stretch). docs/103 (2026-09-06): a single reassert at blank time
		//! measurably "wakes" the companion channel, but the effect decays -- live-tested,
		//! a real don ~100s after a one-shot reassert produced zero new packets, same as
		//! having no reassert at all. This drives a periodic re-arm instead of a one-shot;
		//! see WMR_PRESENCE_REASSERT_INTERVAL_MS in wmr_hmd_update_inputs().
		uint64_t last_reassert_ns;

		/*
		 * TEMPORARY diagnostic counters for docs/98 (reverb-g2, 2026-09-05): auto-standby
		 * BLANK fires correctly but RESTORE never has, live, twice. These exist only to tell
		 * apart "the raw proximity/IPD channel stopped delivering packets at all" from "packets
		 * keep arriving but wmr_hmd_update_inputs() stops being invoked (or stops evaluating
		 * them)" without a rebuild mid-investigation. Gated behind WMR_PRESENCE_DIAG (default
		 * off); remove once RESTORE is root-caused and fixed for real, live-validated.
		 */
		//! Every call to wmr_hmd_update_inputs(), regardless of what it finds. A stall here
		//! (no growth for a long stretch) means the STATE TRACKER stopped invoking us -- not a
		//! wmr_hmd.c bug, but a fact worth telling apart from the channel dying underneath us.
		uint64_t diag_update_inputs_calls;
		//! Every WMR_CONTROL_MSG_IPD_VALUE packet decoded, changed or not. A stall here while
		//! diag_update_inputs_calls keeps climbing means the companion channel itself has gone
		//! quiet -- the raw sensor/transport, not the driver's evaluation of it.
		uint64_t diag_proximity_packets_seen;
		//! Throttle for the periodic diagnostic heartbeat log in wmr_hmd_update_inputs().
		uint64_t diag_last_heartbeat_ns;
	} presence;

	//! docs/103 (2026-09-06, "always evaluate" fix): true iff WMR_USER_PRESENCE=1 was set
	//! at wmr_hmd_create() time. Gates whether wmr_run_thread() calls
	//! wmr_hmd_presence_tick() every loop iteration -- without this, a WMR headset with
	//! the feature off would still pay a mutex lock/unlock per iteration for nothing.
	bool presence_enabled;
	//! Protects every field inside `presence` above. Needed now that the debounce/commit/
	//! blank/restore/reassert decision (formerly done inline in wmr_hmd_update_inputs(),
	//! called only when an OpenXR client is actively syncing frames) moved to
	//! wmr_hmd_presence_tick(), called every iteration of the always-running "WMR: USB-HMD"
	//! thread (wmr_run_thread) instead -- see that function's own comment for why. Only
	//! `committed` is still read from wmr_hmd_update_inputs() (the OpenXR-thread side), so
	//! the critical sections on that side are tiny; the tick function on the read thread
	//! holds it for the whole decision, same shape as the existing controller_status_lock
	//! a few members below.
	struct os_mutex presence_lock;

	struct hololens_sensors_packet packet;

	struct
	{
		//! Protects all members of the `fusion` substruct.
		struct os_mutex mutex;

		//! Main fusion calculator.
		struct m_imu_3dof i3dof;

		//! The last angular velocity from the IMU, for prediction.
		struct xrt_vec3 last_angular_velocity;

		//! When did we get the last IMU sample, in CPU time.
		uint64_t last_imu_timestamp_ns;
	} fusion;

	//! Fields related to camera-based tracking (SLAM and hand tracking)
	struct
	{
		//! Source of video/IMU data for tracking
		struct xrt_fs *source;

		//! Context for @ref source
		struct xrt_frame_context xfctx;

		//! SLAM tracker.
		//! @todo Right now, we are not consistent in how we interface with
		//! trackers. In particular, we have a @ref xrt_tracked_slam field but not
		//! an equivalent for hand tracking.
		struct xrt_tracked_slam *slam;

		//! Calibration data for SLAM
		struct t_slam_calibration slam_calib;

		//! Set at start. Whether the SLAM tracker was initialized.
		bool slam_enabled;

		//! Set at start. Whether the hand tracker was initialized.
		bool hand_enabled;

		//! SLAM systems track the IMU pose, enabling this corrects it to middle of the eyes
		bool imu2me;

		/*!
		 * Optional constellation tracker for controller positional tracking (WMR_CONSTELLATION_CONTROLLERS).
		 * NULL unless the feature is enabled and creation succeeded. Owned by @ref wh, destroyed via @ref
		 * xfctx like everything else here.
		 */
		struct t_constellation_tracker *constellation_tracker;

		//! Per-camera blob sinks handed out by @ref constellation_tracker, indexed like @ref
		//! wmr_hmd_config.tcams. Entries beyond the tracker's camera count are NULL. Only valid when
		//! @ref constellation_tracker is non-NULL.
		struct t_blob_sink *constellation_cam_blob_sinks[WMR_MAX_CAMERAS];

		/*!
		 * Where the constellation tracker gets the headset's own pose from, so it can place the
		 * head-mounted cameras in the world. The mosaic's cameras are given IMU-relative poses; this
		 * is what turns those into world poses every frame. See @ref
		 * wmr_hmd_constellation_tracking_source_get_tracked_pose.
		 */
		struct t_constellation_tracker_tracking_source constellation_tracking_source;
	} tracking;

	//! Whether to track the HMD with 6dof SLAM or fallback to the `fusion` 3dof tracker
	bool slam_over_3dof;

	//! Last tracked pose
	struct xrt_pose pose;

	//! Additional offset to apply to `pose`
	struct xrt_pose offset;

	//! Average 4 IMU samples before sending them to the trackers
	bool average_imus;

	/*!
	 * Offset for tracked pose offsets (applies to both fusion and SLAM).
	 * Applied when getting the tracked poses, so is effectively a offset
	 * to increase or decrease prediction.
	 */
	struct u_var_draggable_f32 tracked_offset_ms;

	struct
	{
		struct u_var_button hmd_screen_enable_btn;
		struct u_var_button switch_tracker_btn;
		char hand_status[128];
		char slam_status[128];
	} gui;

	/* Tunnelled controller devices (Reverb G2, Odyssey+) handling */
	struct os_mutex controller_status_lock;
	struct os_cond controller_status_cond;
	bool have_left_controller_status;
	bool have_right_controller_status;

	struct wmr_hmd_controller_connection *controller[WMR_MAX_CONTROLLERS];
};

static inline struct wmr_hmd *
wmr_hmd(struct xrt_device *p)
{
	return (struct wmr_hmd *)p;
}

void
wmr_hmd_create(enum wmr_headset_type hmd_type,
               struct os_hid_device *hid_holo,
               struct os_hid_device *hid_ctrl,
               struct xrt_prober_device *dev_holo,
               struct xrt_prober_device *dev_companion,
               enum u_logging_level log_level,
               struct xrt_device **out_hmd,
               struct xrt_device **out_handtracker,
               struct xrt_device **out_left_controller,
               struct xrt_device **out_right_controller);

bool
wmr_hmd_send_controller_packet(struct wmr_hmd *hmd, const uint8_t *buffer, uint32_t buf_size);
int
wmr_hmd_read_sync_from_controller(struct wmr_hmd *hmd, uint8_t *buffer, uint32_t buf_size, int timeout_ms);
#ifdef __cplusplus
}
#endif
