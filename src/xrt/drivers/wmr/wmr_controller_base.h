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

	//! Time of last IMU sample, in CPU time.
	uint64_t last_imu_timestamp_ns;
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
