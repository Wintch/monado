// Copyright 2018, Philipp Zabel.
// Copyright 2020-2021, N Madsen.
// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver code for a WMR HMD.
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author nima01 <nima_zero_one@protonmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @author Nova King <technobaboo@proton.me>
 * @ingroup drv_wmr
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_tracking.h"

#include "os/os_time.h"
#include "os/os_hid.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include "math/m_predict.h"
#include "math/m_vec2.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_trace_marker.h"
#include "util/u_distortion_mesh.h"
#include "util/u_sink.h"

#ifdef XRT_OS_LINUX
#include "util/u_linux.h"
#endif

#include "tracking/t_tracking.h"

#include "wmr_hmd.h"
#include "wmr_common.h"
#include "wmr_config_key.h"
#include "wmr_protocol.h"
#include "wmr_source.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#ifdef XRT_OS_LINUX
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#endif
#ifndef XRT_OS_WINDOWS
#include <unistd.h> // for sleep()
#endif

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "../multi_wrapper/multi.h"
#include "../drivers/ht/ht_interface.h"
#endif

// Unsure if these can change nor how to get them if so
#define CAMERA_FREQUENCY 30      //!< Observed value (OV7251)
#define IMU_FREQUENCY 1000       //!< Observed value (ICM20602)
#define IMU_SAMPLES_PER_PACKET 4 //!< There are 4 samples for each USB IMU packet

//! Specifies whether the user wants to use a SLAM tracker.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_slam, "WMR_SLAM", true)

//! Whether to start tracking-camera streaming. With cameras off, SLAM and hand tracking
//! are disabled and the headset runs orientation-only from the IMU. Lets a headset whose
//! cameras fail to start (or whose user needs neither) run at all instead of aborting.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_cameras, "WMR_CAMERAS", true)

//! Specifies whether the user wants to use a SLAM tracker.
DEBUG_GET_ONCE_NUM_OPTION(sleep_seconds, "WMR_DISPLAY_INIT_SLEEP_SECONDS", 4)

//! Specifies whether the user wants to use the hand tracker.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_handtracking, "WMR_HANDTRACKING", true)

//! Whether to run the optical LED constellation tracker for controller positional tracking.
//! Default on since 0017 (patches/monado/0017): controllers report a real fused position, not
//! just orientation. See docs/03-controllers.md.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_constellation_controllers, "WMR_CONSTELLATION_CONTROLLERS", false)

//! Default off. On the SLAM head-pose path, wmr_hmd_get_slam_tracked_pose() always clears
//! XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT before returning, so every consumer downstream
//! of this driver -- including a SteamVR compatibility layer relaying our poses onward -- sees
//! zero head angular velocity regardless of how fast the head is actually turning. SteamVR's own
//! compositor uses TrackedDevicePose_t's velocity fields for its late-stage photon-time
//! extrapolation and motion-smoothing/reprojection warp; hand it zero and that extrapolation is a
//! no-op for rotation, i.e. it loses one full photon-time interval of rotational-latency
//! prediction it would otherwise apply from real data instead of assuming a static head.
//! When on, forwards the angular velocity the SLAM tracker itself already computed for this exact
//! timestamp (see predict_pose() in t_tracker_slam.cpp), axis-corrected into the WMR frame, and
//! sets the valid bit -- see the comment at its use site for why this is NOT simply reusing
//! wh->fusion.last_angular_velocity, and for the prediction-double-counting risk that has NOT been
//! ruled out.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_forward_angular_velocity, "WMR_FORWARD_ANGULAR_VELOCITY", false)

#ifdef XRT_FEATURE_SLAM
//! Whether to submit samples to the SLAM tracker from the start.
DEBUG_GET_ONCE_OPTION(slam_submit_from_start, "SLAM_SUBMIT_FROM_START", NULL)
#endif

//! Specifies the y offset of the views.
DEBUG_GET_ONCE_NUM_OPTION(left_view_y_offset, "WMR_LEFT_DISPLAY_VIEW_Y_OFFSET", 0)
DEBUG_GET_ONCE_NUM_OPTION(right_view_y_offset, "WMR_RIGHT_DISPLAY_VIEW_Y_OFFSET", 0)

//! T211 chair-oscillation test: yaw motion leaks into the delivered roll/pitch axes (fit R2
//! 0.60/0.44 against cumulative yaw, persists after motion stops -- a fixed-direction gyro
//! mounting misalignment, not noise/bias; the factory gyro mix_matrix itself is near-identity,
//! see the "IMU factory calib" INFO line above). Composes a small constant rotation into the
//! gyro transform to cancel it. Default off pending live re-verification with the fix on
//! (docs/pruebas.jsonl T211/T212); see the derivation comment on wmr_hmd_get_imu_calib().
DEBUG_GET_ONCE_BOOL_OPTION(wmr_hmd_gyro_mount_fix, "WMR_HMD_GYRO_MOUNT_FIX", false)

//! Surface the G2's companion-device proximity sensor (docs/12-g2-protocol.md
//! WMR_CONTROL_MSG_IPD_VALUE) as Monado's generic XR_EXT_user_presence input
//! (XRT_INPUT_GENERIC_HEAD_DETECT). Default off: this project's own docs/22 notes the
//! proximity byte's worn/not-worn semantics were never actually confirmed with a clean
//! cover/uncover gesture (the one session that tried it hit the companion device
//! mid-USB2-storm and couldn't read anything), so ship it opt-in until live data
//! validates the threshold in wmr_hmd_update_inputs(). Zero behavior change when unset.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_user_presence, "WMR_USER_PRESENCE", false)

//! Debounce windows for XR_EXT_user_presence, in milliseconds (reverb-g2 T224). Asymmetric on
//! purpose -- see the `presence` struct's comment in wmr_hmd.h. A spurious resume is invisible;
//! a spurious pause interrupts the session, so leaving "worn" costs more evidence than entering
//! it. 0 on either disables that direction's debounce.
DEBUG_GET_ONCE_NUM_OPTION(wmr_user_presence_don_ms, "WMR_USER_PRESENCE_DON_MS", 250)
DEBUG_GET_ONCE_NUM_OPTION(wmr_user_presence_doff_ms, "WMR_USER_PRESENCE_DOFF_MS", 1000)


#define WMR_TRACE(d, ...) U_LOG_XDEV_IFL_T(&d->base, d->log_level, __VA_ARGS__)
#define WMR_DEBUG(d, ...) U_LOG_XDEV_IFL_D(&d->base, d->log_level, __VA_ARGS__)
#define WMR_DEBUG_HEX(d, data, data_size) U_LOG_XDEV_IFL_D_HEX(&d->base, d->log_level, data, data_size)
#define WMR_INFO(d, ...) U_LOG_XDEV_IFL_I(&d->base, d->log_level, __VA_ARGS__)
#define WMR_WARN(d, ...) U_LOG_XDEV_IFL_W(&d->base, d->log_level, __VA_ARGS__)
#define WMR_ERROR(d, ...) U_LOG_XDEV_IFL_E(&d->base, d->log_level, __VA_ARGS__)

static int
wmr_hmd_activate_reverb(struct wmr_hmd *wh);
static void
wmr_hmd_deactivate_reverb(struct wmr_hmd *wh);
static void
wmr_hmd_screen_enable_reverb(struct wmr_hmd *wh, bool enable);
static int
wmr_hmd_activate_odyssey_plus(struct wmr_hmd *wh);
static void
wmr_hmd_deactivate_odyssey_plus(struct wmr_hmd *wh);
static void
wmr_hmd_screen_enable_odyssey_plus(struct wmr_hmd *wh, bool enable);

const struct wmr_headset_descriptor headset_map[] = {
    {WMR_HEADSET_GENERIC, NULL, "Unknown WMR HMD", NULL, NULL, NULL}, /* Catch-all for unknown headsets */
    {WMR_HEADSET_HP_VR1000, "HP Reverb VR Headset VR1000-1xxx", "HP VR1000", NULL, NULL, NULL}, /*! @todo init funcs */
    {WMR_HEADSET_REVERB_G1, "HP Reverb VR Headset VR1000-2xxx", "HP Reverb", wmr_hmd_activate_reverb,
     wmr_hmd_deactivate_reverb, wmr_hmd_screen_enable_reverb},
    {WMR_HEADSET_REVERB_G2, "HP Reverb Virtual Reality Headset G2", "HP Reverb G2", wmr_hmd_activate_reverb,
     wmr_hmd_deactivate_reverb, wmr_hmd_screen_enable_reverb},
    {WMR_HEADSET_SAMSUNG_XE700X3AI, "Samsung Windows Mixed Reality XE700X3AI", "Samsung Odyssey",
     wmr_hmd_activate_odyssey_plus, wmr_hmd_deactivate_odyssey_plus, wmr_hmd_screen_enable_odyssey_plus},
    {WMR_HEADSET_SAMSUNG_800ZAA, "Samsung Windows Mixed Reality 800ZAA", "Samsung Odyssey+",
     wmr_hmd_activate_odyssey_plus, wmr_hmd_deactivate_odyssey_plus, wmr_hmd_screen_enable_odyssey_plus},
    {WMR_HEADSET_LENOVO_EXPLORER, "Lenovo VR-2511N", "Lenovo Explorer", NULL, NULL, NULL},
    {WMR_HEADSET_MEDION_ERAZER_X1000, "Medion Erazer X1000", "Medion Erazer", NULL, NULL, NULL},
    {WMR_HEADSET_DELL_VISOR, "DELL VR118", "Dell Visor", NULL, NULL, NULL},
    {WMR_HEADSET_ACER_AH100, "Acer", "AH100", NULL, NULL, NULL},
    {WMR_HEADSET_ACER_AH101, "Acer", "AH101", NULL, NULL, NULL},
    {WMR_HEADSET_FUJITSU_FMVHDS1, "Fujitsu", "Fujitsu FMVHDS1", NULL, NULL, NULL},
};
const int headset_map_n = sizeof(headset_map) / sizeof(headset_map[0]);


/*
 *
 * Hololens decode packets.
 *
 */

static void
hololens_sensors_decode_packet(struct wmr_hmd *wh,
                               struct hololens_sensors_packet *pkt,
                               const unsigned char *buffer,
                               int size)
{
	WMR_TRACE(wh, " ");

	if (size != 497 && size != 381) {
		WMR_ERROR(wh, "invalid hololens sensor packet size (expected 381 or 497 but got %d)", size);
		return;
	}

	pkt->id = read8(&buffer);
	for (int i = 0; i < 4; i++) {
		pkt->temperature[i] = read16(&buffer);
	}

	for (int i = 0; i < 4; i++) {
		pkt->gyro_timestamp[i] = read64(&buffer);
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 32; j++) {
			pkt->gyro[i][j] = read16(&buffer);
		}
	}

	for (int i = 0; i < 4; i++) {
		pkt->accel_timestamp[i] = read64(&buffer);
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			pkt->accel[i][j] = read32(&buffer);
		}
	}

	for (int i = 0; i < 4; i++) {
		pkt->video_timestamp[i] = read64(&buffer);
	}
}

static void
hololens_ensure_controller(struct wmr_hmd *wh, uint8_t controller_id, uint16_t vid, uint16_t pid)
{
	if (controller_id >= WMR_MAX_CONTROLLERS)
		return;

	if (wh->controller[controller_id] != NULL) {
		return;
	}

	WMR_DEBUG(wh, "Adding controller device %d", controller_id);

	enum xrt_device_type controller_type =
	    controller_id == 0 ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	uint8_t hmd_cmd_base = controller_id == 0 ? 0x5 : 0xd;

	struct wmr_hmd_controller_connection *controller =
	    wmr_hmd_controller_create(wh, hmd_cmd_base, controller_type, vid, pid, wh->log_level);

	os_mutex_lock(&wh->controller_status_lock);
	wh->controller[controller_id] = controller;
	os_mutex_unlock(&wh->controller_status_lock);
}

/*
 *
 * Hololens packets.
 *
 */

static void
hololens_handle_unknown(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	WMR_DEBUG(wh, "Unknown hololens sensors message type: %02x, (%i)", buffer[0], size);
}

static void
hololens_handle_control(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	WMR_DEBUG(wh, "WMR_MS_HOLOLENS_MSG_CONTROL: %02x, (%i)", buffer[0], size);
}

static void
hololens_handle_controller_status_packet(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	if (size < 3) {
		WMR_DEBUG(wh, "Got small packet 0x17 (%i)", size);
		return;
	}

	uint8_t controller_id = buffer[1];
	uint8_t pkt_type = buffer[2];

	switch (pkt_type) {
	case WMR_CONTROLLER_STATUS_UNPAIRED: {
		WMR_TRACE(wh, "Controller %d is not paired", controller_id);
		break;
	}
	case WMR_CONTROLLER_STATUS_OFFLINE: {
		if (size < 7) {
			WMR_TRACE(wh, "Got small controller offline status packet (%i)", size);
			return;
		}

		/* Skip packet type, controller id, presence */
		buffer += 3;

		uint16_t vid = read16(&buffer);
		uint16_t pid = read16(&buffer);
		WMR_TRACE(wh, "Controller %d offline. VID 0x%04x PID 0x%04x", controller_id, vid, pid);
		break;
	}
	case WMR_CONTROLLER_STATUS_ONLINE: {
		if (size < 7) {
			WMR_TRACE(wh, "Got small controller online status packet (%i)", size);
			return;
		}

		/* Skip packet type, controller id, presence */
		buffer += 3;

		uint16_t vid = read16(&buffer);
		uint16_t pid = read16(&buffer);

		if (size >= 10) {
			uint8_t unknown1 = read8(&buffer);
			uint16_t unknown2160 = read16(&buffer);
			WMR_TRACE(wh, "Controller %d online. VID 0x%04x PID 0x%04x val1 %u val2 %u", controller_id, vid,
			          pid, unknown1, unknown2160);
		} else {
			WMR_TRACE(wh, "Controller %d online. VID 0x%04x PID 0x%04x", controller_id, vid, pid);
		}

		hololens_ensure_controller(wh, controller_id, vid, pid);
		break;
	}
	default: //
		WMR_DEBUG(wh, "Unknown controller status packet (%i) type 0x%02x", size, pkt_type);
		break;
	}

	os_mutex_lock(&wh->controller_status_lock);
	if (controller_id == 0)
		wh->have_left_controller_status = true;
	else if (controller_id == 1)
		wh->have_right_controller_status = true;
	if (wh->have_left_controller_status && wh->have_right_controller_status)
		os_cond_signal(&wh->controller_status_cond);
	os_mutex_unlock(&wh->controller_status_lock);
}

static void
hololens_handle_bt_iface_packet(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	int pkt_type;

	if (size < 2)
		return;

	if (size < 6) {
		WMR_DEBUG(wh, "Short Bluetooth interface packet (%d) type 0x%02x", size, buffer[1]);
		return;
	}

	pkt_type = buffer[1];
	if (pkt_type != WMR_BT_IFACE_MSG_DEBUG) {
		WMR_DEBUG(wh, "Unknown Bluetooth interface packet (%d) type 0x%02x", size, pkt_type);
		WMR_DEBUG_HEX(wh, buffer, size);
		return;
	}
	buffer += 2;

	uint16_t tag = read16(&buffer);
	uint16_t msg_len = read16(&buffer);

	if (size < msg_len + 6) {
		WMR_DEBUG(wh, "Bluetooth interface debug packet (%d) too short. tag 0x%x msg len %u", size, tag,
		          msg_len);
		return;
	}

	WMR_DEBUG(wh, "BT debug: tag %d: %.*s", tag, msg_len, buffer);
}

static void
hololens_handle_controller_packet(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	if (size < 45) {
		WMR_TRACE(wh, "Got unknown short controller packet (%i)\n\t%02x", size, buffer[0]);
		return;
	}

	uint8_t packet_id = buffer[0];
	struct wmr_controller_connection *controller = NULL;

	if (packet_id == WMR_MS_HOLOLENS_MSG_LEFT_CONTROLLER) {
		controller = (struct wmr_controller_connection *)wh->controller[0];
	} else if (packet_id == WMR_MS_HOLOLENS_MSG_RIGHT_CONTROLLER) {
		controller = (struct wmr_controller_connection *)wh->controller[1];
	}

	if (controller == NULL)
		return; /* Controller online message not yet seen */

	uint64_t now_ns = os_monotonic_get_ns();
	wmr_controller_connection_receive_bytes(controller, now_ns, (uint8_t *)buffer, size);
}

static void
hololens_handle_debug(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	if (size < 12) {
		WMR_TRACE(wh, "Got short debug packet (%i) 0x%02x", size, buffer[0]);
		return;
	}
	buffer += 1;

	uint32_t magic = read32(&buffer);
	if (magic != WMR_MAGIC) {
		WMR_TRACE(wh, "Debug packet (%i) 0x%02x had strange magic 0x%08x", size, buffer[0], magic);
		return;
	}
	uint32_t timestamp = read32(&buffer);
	uint16_t seq = read16(&buffer);
	uint8_t src_tag = read8(&buffer);
	int msg_len = size - 12;

	WMR_DEBUG(wh, "HMD debug: TS %f seq %u src %d: %.*s", timestamp / 1000.0, seq, src_tag, msg_len, buffer);
}

static void
hololens_handle_sensors_avg(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	// Get the timing as close to reading the packet as possible.
	uint64_t now_ns = os_monotonic_get_ns();

	hololens_sensors_decode_packet(wh, &wh->packet, buffer, size);

	// Use a single averaged sample from all the samples in the packet
	struct xrt_vec3 avg_raw_accel = XRT_VEC3_ZERO;
	struct xrt_vec3 avg_raw_gyro = XRT_VEC3_ZERO;
	for (int i = 0; i < IMU_SAMPLES_PER_PACKET; i++) {
		struct xrt_vec3 a = XRT_VEC3_ZERO;
		struct xrt_vec3 g = XRT_VEC3_ZERO;
		vec3_from_hololens_accel(wh->packet.accel, i, &a);
		vec3_from_hololens_gyro(wh->packet.gyro, i, &g);
		math_vec3_accum(&a, &avg_raw_accel);
		math_vec3_accum(&g, &avg_raw_gyro);
	}
	math_vec3_scalar_mul(1.0f / IMU_SAMPLES_PER_PACKET, &avg_raw_accel);
	math_vec3_scalar_mul(1.0f / IMU_SAMPLES_PER_PACKET, &avg_raw_gyro);

	// Calibrate averaged sample
	struct xrt_vec3 avg_calib_accel = XRT_VEC3_ZERO;
	struct xrt_vec3 avg_calib_gyro = XRT_VEC3_ZERO;
	math_matrix_3x3_transform_vec3(&wh->config.sensors.accel.mix_matrix, &avg_raw_accel, &avg_calib_accel);
	math_matrix_3x3_transform_vec3(&wh->config.sensors.gyro.mix_matrix, &avg_raw_gyro, &avg_calib_gyro);
	math_vec3_accum(&wh->config.sensors.accel.bias_offsets, &avg_calib_accel);
	math_vec3_accum(&wh->config.sensors.gyro.bias_offsets, &avg_calib_gyro);
	math_quat_rotate_vec3(&wh->config.sensors.transforms.P_oxr_acc.orientation, &avg_calib_accel, &avg_calib_accel);
	math_quat_rotate_vec3(&wh->config.sensors.transforms.P_oxr_gyr.orientation, &avg_calib_gyro, &avg_calib_gyro);

	// Fusion tracking
	os_mutex_lock(&wh->fusion.mutex);
	timepoint_ns t = wh->packet.gyro_timestamp[IMU_SAMPLES_PER_PACKET - 1] * WMR_MS_HOLOLENS_NS_PER_TICK;
	m_imu_3dof_update(&wh->fusion.i3dof, t, &avg_calib_accel, &avg_calib_gyro);
	wh->fusion.last_imu_timestamp_ns = now_ns;
	wh->fusion.last_angular_velocity = avg_calib_gyro;
	os_mutex_unlock(&wh->fusion.mutex);

	// SLAM tracking
	wmr_source_push_imu_packet(wh->tracking.source, t, avg_raw_accel, avg_raw_gyro);
}

static void
hololens_handle_sensors_all(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	DRV_TRACE_MARKER();

	// Get the timing as close to reading the packet as possible.
	uint64_t now_ns = os_monotonic_get_ns();

	hololens_sensors_decode_packet(wh, &wh->packet, buffer, size);

	struct xrt_vec3 raw_gyro[IMU_SAMPLES_PER_PACKET];
	struct xrt_vec3 raw_accel[IMU_SAMPLES_PER_PACKET];
	struct xrt_vec3 calib_gyro[IMU_SAMPLES_PER_PACKET];
	struct xrt_vec3 calib_accel[IMU_SAMPLES_PER_PACKET];

	for (int i = 0; i < IMU_SAMPLES_PER_PACKET; i++) {
		struct xrt_vec3 *rg = &raw_gyro[i];
		struct xrt_vec3 *cg = &calib_gyro[i];
		vec3_from_hololens_gyro(wh->packet.gyro, i, rg);
		math_matrix_3x3_transform_vec3(&wh->config.sensors.gyro.mix_matrix, rg, cg);
		math_vec3_accum(&wh->config.sensors.gyro.bias_offsets, cg);
		math_quat_rotate_vec3(&wh->config.sensors.transforms.P_oxr_gyr.orientation, cg, cg);

		struct xrt_vec3 *ra = &raw_accel[i];
		struct xrt_vec3 *ca = &calib_accel[i];
		vec3_from_hololens_accel(wh->packet.accel, i, ra);
		math_matrix_3x3_transform_vec3(&wh->config.sensors.accel.mix_matrix, ra, ca);
		math_vec3_accum(&wh->config.sensors.accel.bias_offsets, ca);
		math_quat_rotate_vec3(&wh->config.sensors.transforms.P_oxr_acc.orientation, ca, ca);
	}

	// Fusion tracking
	os_mutex_lock(&wh->fusion.mutex);
	for (int i = 0; i < IMU_SAMPLES_PER_PACKET; i++) {
		m_imu_3dof_update(                                              //
		    &wh->fusion.i3dof,                                          //
		    wh->packet.gyro_timestamp[i] * WMR_MS_HOLOLENS_NS_PER_TICK, //
		    &calib_accel[i],                                            //
		    &calib_gyro[i]);                                            //
	}
	wh->fusion.last_imu_timestamp_ns = now_ns;
	wh->fusion.last_angular_velocity = calib_gyro[3];
	os_mutex_unlock(&wh->fusion.mutex);

	// SLAM tracking
	for (int i = 0; i < IMU_SAMPLES_PER_PACKET; i++) {
		timepoint_ns t = wh->packet.gyro_timestamp[i] * WMR_MS_HOLOLENS_NS_PER_TICK;
		wmr_source_push_imu_packet(wh->tracking.source, t, raw_accel[i], raw_gyro[i]);
	}
}

static void
hololens_handle_sensors(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	if (wh->average_imus) {
		// Less overhead and jitter.
		hololens_handle_sensors_avg(wh, buffer, size);
	} else {
		// More sophisticated fusion algorithms might work better with raw data.
		hololens_handle_sensors_all(wh, buffer, size);
	}
}

static bool
hololens_sensors_read_packets(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	WMR_TRACE(wh, " ");

	unsigned char buffer[WMR_FEATURE_BUFFER_SIZE];

	// Block for 100ms
	os_mutex_lock(&wh->hid_lock);
	int size = os_hid_read(wh->hid_hololens_sensors_dev, buffer, sizeof(buffer), 100);
	os_mutex_unlock(&wh->hid_lock);

	if (size < 0) {
		// The Hololens Sensors device shares the headset's internal USB2 hub with
		// the companion device, which is already known to transiently drop off and
		// re-enumerate under panel load (see control_read_packets()). A single read
		// error here used to permanently kill this thread -- and with it every IMU
		// sample, the SLAM tracking feed, and the tunnelled controller packets --
		// even though the device itself stays enumerated and healthy (seen live
		// 2026-08-13: one -1, tracking frozen for the rest of the session, device
		// still on the bus). Tolerate a bounded run of consecutive failures
		// instead, same shape as the Bluetooth controller tunnel
		// (wmr_bt_controller.c). Unlike the companion device this feed is
		// load-bearing for tracking, so warn loudly and give up after a bound
		// rather than tolerating forever.
		wh->hololens_consecutive_read_errors++;
		if (wh->hololens_consecutive_read_errors >= 10) {
			WMR_ERROR(wh,
			          "Error reading from Hololens Sensors device: %d consecutive read errors, "
			          "giving up. Call to os_hid_read returned %i",
			          wh->hololens_consecutive_read_errors, size);
			return false;
		}
		WMR_WARN(wh,
		         "Error reading from Hololens Sensors device (%d in a row). Call to os_hid_read returned %i",
		         wh->hololens_consecutive_read_errors, size);
		// The healthy path is paced by the blocking 100ms os_hid_read above, but a
		// dead fd makes poll() return POLLNVAL instantly -- this sleep keeps the
		// failure path from busy-spinning until the bound trips.
		os_nanosleep(U_TIME_1MS_IN_NS * 10);
		return true;
	}
	wh->hololens_consecutive_read_errors = 0;
	if (size == 0) {
		WMR_TRACE(wh, "No more data to read");
		return true; // No more messages, return.
	} else {
		WMR_TRACE(wh, "Read %u bytes", size);
	}

	switch (buffer[0]) {
	case WMR_MS_HOLOLENS_MSG_SENSORS: //
		hololens_handle_sensors(wh, buffer, size);
		break;
	case WMR_MS_HOLOLENS_MSG_BT_IFACE: //
		hololens_handle_bt_iface_packet(wh, buffer, size);
		break;
	case WMR_MS_HOLOLENS_MSG_LEFT_CONTROLLER:
	case WMR_MS_HOLOLENS_MSG_RIGHT_CONTROLLER: //
		hololens_handle_controller_packet(wh, buffer, size);
		break;
	case WMR_MS_HOLOLENS_MSG_CONTROLLER_STATUS: //
		hololens_handle_controller_status_packet(wh, buffer, size);
		break;
	case WMR_MS_HOLOLENS_MSG_CONTROL: //
		hololens_handle_control(wh, buffer, size);
		break;
	case WMR_MS_HOLOLENS_MSG_DEBUG: //
		hololens_handle_debug(wh, buffer, size);
		break;
	default: //
		hololens_handle_unknown(wh, buffer, size);
		break;
	}

	return true;
}


/*
 *
 * Control packets.
 *
 */

static void
control_ipd_value_decode(struct wmr_hmd *wh, const unsigned char *buffer, int size)
{
	if (size != 2 && size != 4) {
		WMR_ERROR(wh, "Invalid control ipd distance packet size (expected 4 but got %i)", size);
		return;
	}

	uint8_t id = read8(&buffer);
	if (id != 0x1) {
		WMR_ERROR(wh, "Invalid control IPD distance packet ID (expected 0x1 but got %u)", id);
		return;
	}

	uint8_t proximity = read8(&buffer);
	uint16_t ipd_value = (size == 4) ? read16(&buffer) : wh->raw_ipd;

	bool changed = (wh->raw_ipd != ipd_value) || (wh->proximity_sensor != proximity);

	wh->raw_ipd = ipd_value;
	wh->proximity_sensor = proximity;
	// Arrival time, not change time: presence needs to know the CHANNEL is alive, and this
	// message is change-driven, so "no update" and "no change" look identical from the value
	// alone. T224 lost the doff measurement twice to exactly that ambiguity -- the companion
	// died mid-gesture and the byte simply never came, which is indistinguishable from a
	// sensor calmly reporting the same thing.
	wh->presence.last_update_ns = os_monotonic_get_ns();

	if (changed) {
		WMR_DEBUG(wh, "Proximity sensor %d IPD: %d", proximity, ipd_value);
	}
}

/*
 *
 * Companion device hot-reconnect.
 *
 * WHY THIS EXISTS, because it is not obvious from the symptom. The companion device sits on
 * the headset's USB2 branch, which re-enumerates constantly: measured on Linux at 0.92-4.63
 * drops per minute with a p50 outage of 3.0 s, and measured on WINDOWS at 3.47/min with a p50
 * of 2.9 s -- same cable, same headset, same machine, OS as the only variable (docs/60, T226).
 * So the dropouts are the link's and we cannot fix them here.
 *
 * What IS ours is what happens next. A re-enumeration invalidates the open hidraw fd forever:
 * the node the driver holds is gone, every subsequent read returns -1, and no amount of
 * retrying will ever succeed again. That is why the companion error counter keeps climbing
 * through stretches where lsusb already shows the branch back at 5/5 (T183) -- past the first
 * re-enumeration, that counter is measuring our own dead-fd polling, not the device. And
 * everything that rides this channel goes with it for the rest of the session: panel control
 * (the screen-off at shutdown silently does nothing), the IPD value, and the proximity sensor
 * behind XR_EXT_user_presence -- which is exactly the "presence freezes when the storm kills
 * the channel" behaviour, and why the only known cure was relaunching the service.
 *
 * Windows takes the same 274 disconnects in 79 minutes while five titles play and the wearer
 * notices nothing but audio. The difference is not tolerance of a broken handle -- it is that
 * its stack re-opens the device. This does that.
 *
 * Deliberately scoped: only the companion. The USB3 branch (hololens sensors: IMU, camera
 * tunnel, controller tunnel) has never dropped in any capture on either OS, so nothing that
 * matters for tracking passes through here, and a reconnect must never be able to disturb it.
 */

#ifdef XRT_OS_LINUX
/*!
 * Find the hidraw node that currently belongs to the companion device, by the VID/PID
 * remembered at creation time.
 *
 * Deliberately NOT done by remembering the old /dev/hidrawN path: the whole point is that the
 * device re-enumerated, and the kernel hands out a different node number when it does.
 * Deliberately NOT done through the prober either: re-probing rebuilds the device list other
 * drivers hold pointers into, which is a much larger hammer than reading one sysfs attribute.
 */
static bool
companion_find_hidraw_path(struct wmr_hmd *wh, char *out_path, size_t out_path_len)
{
	char want[64];
	// Format used by the HID core in uevent: bus:vendor:product, e.g. 0003:000003F0:00000580.
	snprintf(want, sizeof(want), "HID_ID=0003:%08X:%08X", wh->companion_vid, wh->companion_pid);

	DIR *dir = opendir("/sys/class/hidraw");
	if (dir == NULL) {
		return false;
	}

	bool found = false;
	char fallback[PATH_MAX] = {0};
	struct dirent *ent = NULL;
	while ((ent = readdir(dir)) != NULL) {
		if (strncmp(ent->d_name, "hidraw", 6) != 0) {
			continue;
		}

		char uevent[PATH_MAX];
		snprintf(uevent, sizeof(uevent), "/sys/class/hidraw/%s/device/uevent", ent->d_name);
		FILE *f = fopen(uevent, "r");
		if (f == NULL) {
			continue;
		}

		bool match = false;
		char line[256];
		while (fgets(line, sizeof(line), f) != NULL) {
			if (strncmp(line, want, strlen(want)) == 0) {
				match = true;
				break;
			}
		}
		fclose(f);

		if (!match) {
			continue;
		}

		char dev_path[PATH_MAX];
		snprintf(dev_path, sizeof(dev_path), "/dev/%s", ent->d_name);

		// A VID/PID can carry several HID interfaces. The companion's control interface is
		// USB interface 0 (see wmr_create_headset), so prefer the node whose sysfs path
		// goes through a ":1.0" interface directory, and only fall back to a bare VID/PID
		// match if none of them says so.
		char link[PATH_MAX];
		snprintf(link, sizeof(link), "/sys/class/hidraw/%s/device", ent->d_name);
		char real[PATH_MAX];
		if (realpath(link, real) != NULL && strstr(real, ":1.0/") != NULL) {
			snprintf(out_path, out_path_len, "%s", dev_path);
			found = true;
			break;
		}
		if (fallback[0] == '\0') {
			snprintf(fallback, sizeof(fallback), "%s", dev_path);
		}
	}
	closedir(dir);

	if (!found && fallback[0] != '\0') {
		snprintf(out_path, out_path_len, "%s", fallback);
		found = true;
	}

	return found;
}

/*!
 * Try once to replace the dead companion handle with a fresh one.
 *
 * Returns true only when the swap actually happened. A false is the ORDINARY case during a
 * 2-3 s outage -- the device simply is not there yet -- and is not logged loudly.
 */
static bool
wmr_hmd_companion_reconnect(struct wmr_hmd *wh)
{
	char path[PATH_MAX];
	int64_t f0 = os_monotonic_get_ns(); // T244: time the scan and the open separately (one 3.0 s
	                                     // stall in 75 reconnects was left unexplained by the loop instrument)
	bool present = companion_find_hidraw_path(wh, path, sizeof(path));
	int64_t f1 = os_monotonic_get_ns();
	if (f1 - f0 > 20 * U_TIME_1MS_IN_NS) {
		WMR_WARN(wh, "reconnect: hidraw scan took %.1f ms (present=%d)", (double)(f1 - f0) / 1e6, present);
	}
	if (!present) {
		// Still absent. Expected for the duration of the outage.
		return false;
	}

	struct os_hid_device *fresh = NULL;
	int ret = os_hid_open_hidraw(path, &fresh);
	int64_t f2 = os_monotonic_get_ns();
	if (f2 - f1 > 20 * U_TIME_1MS_IN_NS) {
		WMR_WARN(wh, "reconnect: open(%s) took %.1f ms (ret=%d)", path, (double)(f2 - f1) / 1e6, ret);
	}
	if (ret != 0 || fresh == NULL) {
		wh->companion_reconnect_failures++;
		// EACCES here is systematic, not transient: the node reappeared before udev
		// re-applied its permissions, or the udev rule is missing entirely. Say so
		// once rather than hiding it in a retry loop that will never succeed.
		if (wh->companion_reconnect_failures == 1 || wh->companion_reconnect_failures % 100 == 0) {
			WMR_WARN(wh, "Companion device is present at %s but could not be opened: %i (%s)", path, ret,
			         strerror(ret < 0 ? -ret : ret));
		}
		return false;
	}

	uint64_t now_ns = os_monotonic_get_ns();
	uint64_t dead_ms =
	    wh->companion_dead_since_ns != 0 ? (now_ns - wh->companion_dead_since_ns) / U_TIME_1MS_IN_NS : 0;

	// Swap and close under the lock, so no other thread can still be inside a call on the
	// old handle. Every companion access reads wh->hid_control_dev with hid_lock held --
	// that is what makes this safe, and why the HID_SEND/HID_GET call sites below pass the
	// field itself rather than a pointer cached before the lock.
	os_mutex_lock(&wh->hid_lock);
	struct os_hid_device *stale = wh->hid_control_dev;
	wh->hid_control_dev = fresh;
	if (stale != NULL) {
		os_hid_destroy(stale);
	}
	os_mutex_unlock(&wh->hid_lock);

	wh->companion_consecutive_read_errors = 0;
	wh->companion_backoff_until_ns = 0;
	wh->companion_dead_since_ns = 0;
	wh->companion_reconnect_count++;

	WMR_INFO(wh, "Companion device RECONNECTED on %s after %" PRIu64 " ms dead (reconnect #%u, %u failed opens)",
	         path, dead_ms, wh->companion_reconnect_count, wh->companion_reconnect_failures);

	// Re-assert the panel state. A companion power cycle is exactly the case where the
	// screen-enable command is lost, and the historical symptom of that is a panel that
	// stays dark for the rest of the session with a relaunch as the only cure. Sending it
	// again is the same recovery the activation path already performs for the same reason
	// (see wmr_hmd_activate_reverb's second screen_enable call). Env-gated in case a rig
	// ever turns out to dislike it: WMR_COMPANION_RECONNECT_SCREEN=0.
	static int reassert_screen = -1;
	if (reassert_screen == -1) {
		const char *env = getenv("WMR_COMPANION_RECONNECT_SCREEN");
		reassert_screen = (env == NULL || env[0] != '0');
	}
	if (reassert_screen && wh->hmd_desc != NULL && wh->hmd_desc->screen_enable_func != NULL) {
		int64_t se0 = os_monotonic_get_ns();
		wh->hmd_desc->screen_enable_func(wh, wh->hmd_screen_enable);
		WMR_INFO(wh, "reconnect: screen re-assert took %.1f ms", (double)(os_monotonic_get_ns() - se0) / 1e6);
	}

	// Proximity/IPD re-sync after a reconnect: OFF by default since 2026-08-21 (T244).
	// 0090 added a feature-report read of WMR_CONTROL_MSG_IPD_VALUE here as "the experiment
	// that finds out" whether the device answers. Measured: it does NOT answer, and the read
	// blocks for 1.4-5.0 s (the usbhid control-transfer timeout) INSIDE the shared run loop,
	// which is also the IMU reader -- so every companion re-enumeration stalled the IMU and
	// camera streams by up to 5 s (kernel hidraw ring full, headset still sending), and the
	// backlog then dragged the hw2mono filter (see wmr_source.c) into a 3.5 s rejection
	// hole. The wearer got relocated on every natural USB2 drop. WMR_COMPANION_RECONNECT_RESYNC=1
	// re-enables the read for a future A/B; presence stays on its pre-outage value until the
	// sensor next changes, which is the lesser evil by a wide margin.
	static int resync_enabled = -1;
	if (resync_enabled == -1) {
		const char *env = getenv("WMR_COMPANION_RECONNECT_RESYNC");
		resync_enabled = (env != NULL && env[0] == '1');
	}
	if (resync_enabled) {
		// Try to re-sync the proximity/IPD value. This message is CHANGE-driven, so after a
		// reconnect the driver can sit indefinitely on a value from before the outage -- if the
		// wearer took the headset off during it, presence would be wrong until they moved it
		// again. Whether the device answers a feature-report read of this id is unknown (the
		// activation path does exactly this for ids 0x50/0x09/0x08/0x06), so this is also the
		// experiment that finds out; the answer is logged either way and costs one syscall.
		unsigned char resync[64] = {WMR_CONTROL_MSG_IPD_VALUE};
		int64_t rs0 = os_monotonic_get_ns();
		os_mutex_lock(&wh->hid_lock);
		int rret = os_hid_get_feature(wh->hid_control_dev, resync[0], resync, sizeof(resync));
		os_mutex_unlock(&wh->hid_lock);
		WMR_INFO(wh, "reconnect: proximity feature read took %.1f ms", (double)(os_monotonic_get_ns() - rs0) / 1e6);
		if (rret >= 3 && resync[0] == WMR_CONTROL_MSG_IPD_VALUE) {
			WMR_INFO(wh, "Companion proximity re-synced by feature read after reconnect (%i bytes)", rret);
			control_ipd_value_decode(wh, resync, rret);
		} else {
			WMR_INFO(wh,
			         "Companion proximity could NOT be re-synced by feature read (returned %i); presence stays "
			         "on its pre-outage value until the sensor next changes",
			         rret);
		}
	}
	return true;
}
#endif // XRT_OS_LINUX

static bool
control_read_packets(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	unsigned char buffer[WMR_FEATURE_BUFFER_SIZE];

	// 0049 follow-up (2026-08-17, docs/44, T199): the backoff below used to be a 10ms
	// os_nanosleep INSIDE this shared read loop. With the (universal) companion storm
	// active, that capped the whole loop -- hololens sensors reads included -- at
	// ~100 iterations/s against ~250 IMU packets/s produced, so the kernel-side ring
	// filled and the IMU stream ran a pinned ~630ms stale. hw2mono (fit from IMU
	// arrivals) absorbed the lag, pushing the camera stamps ~630ms into the FUTURE:
	// that one number is the 632-666ms "magic number" behind the T192/T194 collapse
	// (docs/39's image-ahead-of-IMU stall) and the constant ~1s perceived latency
	// (prediction trusts the stamps, so it cannot bridge staleness it cannot see).
	// Back off by SKIPPING the companion attempt until a deadline instead: same
	// <=100Hz retry ceiling for the companion, zero impact on the loop's pace.
	if (wh->companion_backoff_until_ns != 0 && os_monotonic_get_ns() < wh->companion_backoff_until_ns) {
		return true;
	}

	// Do not block
	os_mutex_lock(&wh->hid_lock);
	int size = os_hid_read(wh->hid_control_dev, buffer, sizeof(buffer), 0);
	os_mutex_unlock(&wh->hid_lock);

	if (size < 0) {
		// The companion device can transiently drop off and re-enumerate while the
		// headset display is powering up. Don't kill the whole read thread over a
		// read error here: the same thread also reads the hololens sensors device
		// (IMU and tunnelled controller packets), which is typically still healthy.
		//
		// This read is non-blocking, so in isolation a single failure is one cheap
		// syscall. But a *sustained* dropout is not rare -- confirmed under real
		// gameplay load (SLAM + constellation + app, docs/pruebas.jsonl T188) --
		// and the assumption that the outer loop is always paced by the hololens
		// device's blocking 100ms read doesn't hold once real IMU data is arriving
		// fast enough that read to stop blocking. Left unbounded, that measured
		// 472175 consecutive failures and monado-service pinned at 400%+ CPU for
		// the whole ~17-minute session, recovering only on SIGTERM. Back off once
		// a dropout looks sustained rather than transient. Unlike the hololens
		// sensors read, never give up -- the companion device isn't load-bearing
		// for tracking, so slowing the retry rate is enough.
		wh->companion_consecutive_read_errors++;
		if (wh->companion_consecutive_read_errors == 1 || wh->companion_consecutive_read_errors % 1000 == 0) {
			WMR_WARN(wh,
			         "Error reading from companion (HMD control) device (%d in a row). Call to "
			         "os_hid_read returned %i",
			         wh->companion_consecutive_read_errors, size);
		} else {
			WMR_DEBUG(wh,
			          "Error reading from companion (HMD control) device. Call to os_hid_read returned %i",
			          size);
		}
#ifdef XRT_OS_LINUX
		// Remember when this dead stretch started, so a recovery can be reported with a
		// number instead of an adjective.
		if (wh->companion_dead_since_ns == 0) {
			wh->companion_dead_since_ns = os_monotonic_get_ns();
		}

		// Past the point where this stopped looking like a hiccup, assume the handle is
		// a corpse and go looking for the device's new node.
		//
		// Both constants are TIME, not error counts, on purpose: the error count accrues
		// at whatever rate the shared read loop happens to be running (paced by the
		// hololens device's blocking read, and by this function's own backoff), so a
		// count threshold makes the recovery latency depend on unrelated load. 500 ms of
		// silence is comfortably longer than any hiccup and far shorter than the ~3 s
		// outages the link actually produces; retrying every 250 ms costs four scans of
		// /sys/class/hidraw per second of downtime and nothing at all while healthy.
		// WMR_COMPANION_RECONNECT=0 restores the old retry-the-dead-fd-forever behaviour
		// for an A/B.
		static int reconnect_enabled = -1;
		if (reconnect_enabled == -1) {
			const char *env = getenv("WMR_COMPANION_RECONNECT");
			reconnect_enabled = (env == NULL || env[0] != '0');
		}
		uint64_t now_ns = os_monotonic_get_ns();
		bool dead_long_enough = (now_ns - wh->companion_dead_since_ns) > (U_TIME_1MS_IN_NS * 500);
		if (reconnect_enabled && dead_long_enough && now_ns >= wh->companion_reconnect_next_ns) {
			wh->companion_reconnect_next_ns = now_ns + U_TIME_1MS_IN_NS * 250;
			if (wmr_hmd_companion_reconnect(wh)) {
				return true;
			}
		}
#endif

		if (wh->companion_consecutive_read_errors > 50) {
			// WMR_COMPANION_BACKOFF_BLOCKING=1 restores the old in-loop sleep for
			// an A/B against the skip-based backoff (see the comment at the top of
			// this function for why the sleep is the wrong shape).
			static int backoff_blocking = -1;
			if (backoff_blocking == -1) {
				backoff_blocking = getenv("WMR_COMPANION_BACKOFF_BLOCKING") != NULL;
			}
			if (backoff_blocking) {
				os_nanosleep(U_TIME_1MS_IN_NS * 10);
			} else {
				wh->companion_backoff_until_ns = os_monotonic_get_ns() + U_TIME_1MS_IN_NS * 10;
			}
		}
		return true;
	}
	wh->companion_consecutive_read_errors = 0;
	wh->companion_backoff_until_ns = 0;
	wh->companion_dead_since_ns = 0;
	if (size == 0) {
		WMR_TRACE(wh, "No more data to read");
		return true; // No more messages, return.
	} else {
		WMR_TRACE(wh, "Read %u bytes", size);
	}

	DRV_TRACE_IDENT(control_packet_got);

	switch (buffer[0]) {
	case WMR_CONTROL_MSG_IPD_VALUE: //
		control_ipd_value_decode(wh, buffer, size);
		break;
	case WMR_CONTROL_MSG_UNKNOWN_02: //
		WMR_DEBUG(wh, "Unknown message type: %02x (size %i)", buffer[0], size);
		if (size == 4) {
			// Todo: Decode.
			// On Reverb G1 this message is sometimes received right after a
			// proximity/IPD message, and it always seems to be '02 XX 0d 26'.
			WMR_DEBUG(wh, "---> Type and content bytes: %02x %02x %02x %02x", buffer[0], buffer[1],
			          buffer[2], buffer[3]);
		}
		break;
	case WMR_CONTROL_MSG_DEVICE_STATUS: //
		WMR_DEBUG(wh, "Device status message type: %02x (size %i)", buffer[0], size);
		if (size != 11) {
			WMR_DEBUG(wh,
			          "---> Unexpected message size. Expected 11 bytes incl. message type. Got %d bytes",
			          size);
			WMR_DEBUG_HEX(wh, buffer, size);
			if (size < 11) {
				break;
			}
		}

		// Todo: HMD state info to be decoded further.
		// On Reverb G1 this message is received twice after having sent an 'enable screen' command to the HMD
		// companion device. The first one is received promptly. The second one is received a few seconds later
		// once the HMD screen backlight visibly powers on.
		// 1st message: '05 00 01 01 00 00 00 00 00 00 00'
		// 2nd message: '05 01 01 01 01 00 00 00 00 00 00'
		WMR_DEBUG(wh, "---> Type and content bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		          buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7],
		          buffer[8], buffer[9], buffer[10]);
		WMR_DEBUG(wh,
		          "---> Flags decoded so far: [type: %02x] [display_ready: %02x] [?] [?] [display_ready: %02x] "
		          "[?] [?] [?] [?] [?] [?]",
		          buffer[0], buffer[1], buffer[4]);

		break;
	default: //
		WMR_DEBUG(wh, "Unknown message type: %02x (size %i)", buffer[0], size);
		WMR_DEBUG_HEX(wh, buffer, size);
		break;
	}

	return true;
}


/*
 *
 * Helpers and internal functions.
 *
 */

//! Drive the WMR_CONTROLLER_KEEPALIVE_S v2 tick (wmr_controller_base.c) for both tunnelled
//! controllers, once per wmr_run_thread loop iteration. This thread runs for the entire life of
//! the HMD device regardless of whether any OpenXR client is connected -- see the call site in
//! wmr_run_thread and wmr_controller_base_send_keepalive_if_due()'s own comment for why that
//! property is exactly what v1 (driven from get_tracked_pose) was missing.
static void
wmr_hmd_send_controller_keepalives(struct wmr_hmd *wh)
{
	// controller_status_lock also guards these two pointers against hololens_ensure_controller
	// (same thread, but taken there too -- stay consistent) and against wmr_hmd_destroy (a
	// different thread, though by the time it touches wh->controller[] this thread has already
	// been joined via os_thread_helper_destroy, so that case can't actually race here).
	os_mutex_lock(&wh->controller_status_lock);
	struct wmr_hmd_controller_connection *left = wh->controller[0];
	struct wmr_hmd_controller_connection *right = wh->controller[1];
	os_mutex_unlock(&wh->controller_status_lock);

	if (left != NULL) {
		struct xrt_device *xdev = wmr_hmd_controller_connection_get_controller(left);
		if (xdev != NULL) {
			wmr_controller_base_send_keepalive_if_due(xdev);
		}
	}
	if (right != NULL) {
		struct xrt_device *xdev = wmr_hmd_controller_connection_get_controller(right);
		if (xdev != NULL) {
			wmr_controller_base_send_keepalive_if_due(xdev);
		}
	}
}

static void *
wmr_run_thread(void *ptr)
{
	struct wmr_hmd *wh = (struct wmr_hmd *)ptr;

	U_TRACE_SET_THREAD_NAME("WMR: USB-HMD");
	os_thread_helper_name(&wh->oth, "WMR: USB-HMD");

#ifdef XRT_OS_LINUX
	// Try to raise priority of this thread.
	// T194 experiment: WMR_HMD_THREAD_NO_RT=1 skips this to test whether SCHED_FIFO max
	// priority combined with 0049's 10ms backoff sleep is what starves Basalt's own
	// threads (see docs/pruebas.jsonl T194). Temporary, not a real fix either way.
	if (getenv("WMR_HMD_THREAD_NO_RT") == NULL) {
		u_linux_try_to_set_realtime_priority_on_thread(wh->log_level, "WMR: USB-HMD");
	}

	// T204 round-2 item: pair with XRT_COMPOSITOR_CPU_AFFINITY (comp_multi_system.c and
	// friends) to partition cores between the compositor and the WMR/tracking side instead
	// of only fighting over priority. No-op unless WMR_CPU_AFFINITY is set (e.g. "2,3,4,5").
	u_linux_try_to_set_thread_affinity_from_env(wh->log_level, "WMR: USB-HMD", "WMR_CPU_AFFINITY");
#endif


	os_thread_helper_lock(&wh->oth);
	while (os_thread_helper_is_running_locked(&wh->oth)) {
		os_thread_helper_unlock(&wh->oth);

		// Does not block.
		int64_t t0 = os_monotonic_get_ns(); // T244 run-loop stall instrument
		if (!control_read_packets(wh)) {
			break;
		}
		int64_t t1 = os_monotonic_get_ns();

		// Does block for a bit.
		if (!hololens_sensors_read_packets(wh)) {
			break;
		}
		int64_t t2 = os_monotonic_get_ns();
		if (t1 - t0 > 50 * U_TIME_1MS_IN_NS) {
			WMR_WARN(wh, "run loop: control_read_packets blocked %.1f ms", (double)(t1 - t0) / 1e6);
		}
		if (t2 - t1 > 150 * U_TIME_1MS_IN_NS) {
			WMR_WARN(wh, "run loop: hololens_sensors_read_packets blocked %.1f ms", (double)(t2 - t1) / 1e6);
		}

		// WMR_CONTROLLER_KEEPALIVE_S v2: client-independent, ticks every loop iteration.
		// No locks are held here (both read_packets calls above only hold hid_lock briefly
		// around the raw HID read itself) -- see wmr_hmd_send_controller_keepalives.
		wmr_hmd_send_controller_keepalives(wh);
		int64_t t3 = os_monotonic_get_ns();
		if (t3 - t2 > 50 * U_TIME_1MS_IN_NS) {
			WMR_WARN(wh, "run loop: controller keepalives blocked %.1f ms", (double)(t3 - t2) / 1e6);
		}

		os_thread_helper_lock(&wh->oth);
	}
	os_thread_helper_unlock(&wh->oth);

	WMR_DEBUG(wh, "Exiting reading thread.");

	return NULL;
}

static void
hololens_sensors_enable_imu(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	os_mutex_lock(&wh->hid_lock);
	int size = os_hid_write(wh->hid_hololens_sensors_dev, hololens_sensors_imu_on, sizeof(hololens_sensors_imu_on));
	os_mutex_unlock(&wh->hid_lock);

	if (size <= 0) {
		WMR_ERROR(wh, "Error writing to device");
		return;
	}
}

// The HID argument is evaluated INSIDE the lock on purpose: hid_control_dev can be swapped
// underneath us by wmr_hmd_companion_reconnect(), and a handle read before taking the lock
// could be the one that reconnect is about to close. Callers pass the field, not a cached copy.
#define HID_SEND(hmd, HID, DATA, STR)                                                                                  \
	do {                                                                                                           \
		os_mutex_lock(&hmd->hid_lock);                                                                         \
		struct os_hid_device *_hid = (HID);                                                                    \
		int _ret = _hid != NULL ? os_hid_set_feature(_hid, DATA, sizeof(DATA)) : -1;                            \
		os_mutex_unlock(&hmd->hid_lock);                                                                       \
		if (_ret < 0) {                                                                                        \
			WMR_ERROR(wh, "Send (%s): %i", STR, _ret);                                                     \
		}                                                                                                      \
	} while (false);

#define HID_GET(hmd, HID, DATA, STR)                                                                                   \
	do {                                                                                                           \
		os_mutex_lock(&hmd->hid_lock);                                                                         \
		struct os_hid_device *_hid = (HID);                                                                    \
		int _ret = _hid != NULL ? os_hid_get_feature(_hid, DATA[0], DATA, sizeof(DATA)) : -1;                   \
		os_mutex_unlock(&hmd->hid_lock);                                                                       \
		if (_ret < 0) {                                                                                        \
			WMR_ERROR(wh, "Get (%s): %i", STR, _ret);                                                      \
		} else {                                                                                               \
			WMR_DEBUG(wh, "0x%02x HID feature returned", DATA[0]);                                         \
			WMR_DEBUG_HEX(wh, DATA, _ret);                                                                 \
		}                                                                                                      \
	} while (false);

static int
wmr_hmd_activate_reverb(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();


	WMR_TRACE(wh, "Activating HP Reverb G1/G2 HMD...");

	// Hack to power up the Reverb G1 display, thanks to OpenHMD contributors.
	// Sleep before we start seems to improve reliability.
	// 300ms is what Windows seems to do, so cargo cult that.
	os_nanosleep(U_TIME_1MS_IN_NS * 300);

	for (int i = 0; i < 4; i++) {
		unsigned char cmd[64] = {0x50, 0x01};
		HID_SEND(wh, wh->hid_control_dev, cmd, "loop");

		unsigned char data[64] = {0x50};
		HID_GET(wh, wh->hid_control_dev, data, "loop");

		os_nanosleep(U_TIME_1MS_IN_NS * 10); // Sleep 10ms
	}

	unsigned char data[64] = {0x09};
	HID_GET(wh, wh->hid_control_dev, data, "data_1");

	data[0] = 0x08;
	HID_GET(wh, wh->hid_control_dev, data, "data_2");

	data[0] = 0x06;
	HID_GET(wh, wh->hid_control_dev, data, "data_3");

	WMR_INFO(wh, "Sent activation report.");

	// Enable the HMD screen now, if required. Otherwise, if screen should initially be disabled, then
	// proactively disable it now. Why? Because some cases of irregular termination of Monado will
	// leave either the 'Hololens Sensors' device or its 'companion' device alive across restarts.
	wmr_hmd_screen_enable_reverb(wh, wh->hmd_screen_enable);

	// Allow time for enumeration of available displays by host system, so the compositor can select among them.
	WMR_INFO(wh,
	         "Sleep until the HMD display is powered up, so the available displays can be enumerated by the host "
	         "system.");

	// Get the sleep amount, then sleep. One or two seconds was not enough.
	uint64_t seconds = debug_get_num_option_sleep_seconds();
	os_nanosleep(U_TIME_1S_IN_NS * seconds);

	// The companion HID device can transiently drop off and re-enumerate right around
	// activation, which can silently lose the screen-enable command sent above. Resend it
	// now that things have had time to settle, so the panel doesn't stay stuck off/blank.
	wmr_hmd_screen_enable_reverb(wh, wh->hmd_screen_enable);

	return 0;
}

static void
wmr_hmd_refresh_debug_gui(struct wmr_hmd *wh)
{
	// Update debug GUI button labels.
	if (wh) {
		struct u_var_button *btn = &wh->gui.hmd_screen_enable_btn;
		snprintf(btn->label, sizeof(btn->label),
		         wh->hmd_screen_enable ? "HMD Screen [On]" : "HMD Screen [Off]");
	}
}

static void
wmr_hmd_deactivate_reverb(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	// Turn the screen off
	wmr_hmd_screen_enable_reverb(wh, false);

	//! @todo Power down IMU, and maybe more.
}

static void
wmr_hmd_screen_enable_reverb(struct wmr_hmd *wh, bool enable)
{
	DRV_TRACE_MARKER();


	unsigned char cmd[2] = {0x04, 0x00};
	if (enable) {
		cmd[1] = enable ? 0x01 : 0x00;
	}

	HID_SEND(wh, wh->hid_control_dev, cmd, (enable ? "screen_on" : "screen_off"));

	wh->hmd_screen_enable = enable;

	// Update debug GUI button labels.
	wmr_hmd_refresh_debug_gui(wh);
}

static int
wmr_hmd_activate_odyssey_plus(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();


	WMR_TRACE(wh, "Activating Odyssey HMD...");

	os_nanosleep(U_TIME_1MS_IN_NS * 300);

	unsigned char data[64] = {0x16};
	HID_GET(wh, wh->hid_control_dev, data, "data_1");

	data[0] = 0x15;
	HID_GET(wh, wh->hid_control_dev, data, "data_2");

	data[0] = 0x14;
	HID_GET(wh, wh->hid_control_dev, data, "data_3");

	// Enable the HMD screen now, if required. Otherwise, if screen should initially be disabled, then
	// proactively disable it now. Why? Because some cases of irregular termination of Monado will
	// leave either the 'Hololens Sensors' device or its 'companion' device alive across restarts.
	wmr_hmd_screen_enable_odyssey_plus(wh, wh->hmd_screen_enable);

	// Allow time for enumeration of available displays by host system, so the compositor can select among them.
	WMR_INFO(wh,
	         "Sleep until the HMD display is powered up, so the available displays can be enumerated by the host "
	         "system.");

	os_nanosleep(3LL * U_TIME_1S_IN_NS);

	return 0;
}

static void
wmr_hmd_deactivate_odyssey_plus(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	// Turn the screen off
	wmr_hmd_screen_enable_odyssey_plus(wh, false);

	//! @todo Power down IMU, and maybe more.
}

static void
wmr_hmd_screen_enable_odyssey_plus(struct wmr_hmd *wh, bool enable)
{
	DRV_TRACE_MARKER();


	unsigned char cmd[2] = {0x12, 0x00};
	if (enable) {
		cmd[1] = enable ? 0x01 : 0x00;
	}

	HID_SEND(wh, wh->hid_control_dev, cmd, (enable ? "screen_on" : "screen_off"));

	wh->hmd_screen_enable = enable;

	// Update debug GUI button labels.
	wmr_hmd_refresh_debug_gui(wh);
}

static void
wmr_hmd_screen_enable_toggle(void *wh_ptr)
{
	struct wmr_hmd *wh = (struct wmr_hmd *)wh_ptr;
	if (wh && wh->hmd_desc && wh->hmd_desc->screen_enable_func) {
		wh->hmd_desc->screen_enable_func(wh, !wh->hmd_screen_enable);
	}
}

/*
 *
 * Config functions.
 *
 */

static int
wmr_config_command_sync(struct wmr_hmd *wh, unsigned char type, unsigned char *buf, int len)
{
	DRV_TRACE_MARKER();

	struct os_hid_device *hid = wh->hid_hololens_sensors_dev;

	unsigned char cmd[64] = {0x02, type};
	os_hid_write(hid, cmd, sizeof(cmd));

	do {
		int size = os_hid_read(hid, buf, len, 100);
		if (size < 1) {
			return -1;
		}
		if (buf[0] == WMR_MS_HOLOLENS_MSG_CONTROL) {
			return size;
		}
	} while (true);

	return -1;
}

static int
wmr_read_config_part(struct wmr_hmd *wh, unsigned char type, unsigned char *data, int len)
{
	DRV_TRACE_MARKER();

	unsigned char buf[33];
	int offset = 0;
	int size;

	size = wmr_config_command_sync(wh, 0x0b, buf, sizeof(buf));
	if (size != 33 || buf[0] != 0x02) {
		WMR_ERROR(wh, "Failed to issue command 0b: %02x %02x %02x", buf[0], buf[1], buf[2]);
		return -1;
	}

	size = wmr_config_command_sync(wh, type, buf, sizeof(buf));
	if (size != 33 || buf[0] != 0x02) {
		WMR_ERROR(wh, "Failed to issue command %02x: %02x %02x %02x", type, buf[0], buf[1], buf[2]);
		return -1;
	}

	while (true) {
		size = wmr_config_command_sync(wh, 0x08, buf, sizeof(buf));
		if (size != 33 || (buf[1] != 0x01 && buf[1] != 0x02)) {
			WMR_ERROR(wh, "Failed to issue command 08: %02x %02x %02x", buf[0], buf[1], buf[2]);
			return -1;
		}

		if (buf[1] != 0x01) {
			break;
		}

		if (buf[2] > len || offset + buf[2] > len) {
			WMR_ERROR(wh, "Getting more information then requested");
			return -1;
		}

		memcpy(data + offset, buf + 3, buf[2]);
		offset += buf[2];
	}

	return offset;
}

XRT_MAYBE_UNUSED static int
wmr_read_config_raw(struct wmr_hmd *wh, uint8_t **out_data, size_t *out_size)
{
	DRV_TRACE_MARKER();

	unsigned char meta[84];
	uint8_t *data;
	int size;
	int data_size;

	size = wmr_read_config_part(wh, 0x06, meta, sizeof(meta));
	WMR_DEBUG(wh, "(0x06, meta) => %d", size);

	if (size < 0) {
		return -1;
	}

	/*
	 * No idea what the other 64 bytes of metadata are, but the first two
	 * seem to be little endian size of the data store.
	 */
	data_size = meta[0] | (meta[1] << 8);
	data = calloc(1, data_size + 1);
	if (!data) {
		return -1;
	}
	data[data_size] = '\0';

	size = wmr_read_config_part(wh, 0x04, data, data_size);
	WMR_DEBUG(wh, "(0x04, data) => %d", size);
	if (size < 0) {
		free(data);
		return -1;
	}

	WMR_DEBUG(wh, "Read %d-byte config data", data_size);

	*out_data = data;
	*out_size = size;

	return 0;
}

static int
wmr_read_config(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	unsigned char *data = NULL;
	unsigned char *config_json_block;
	size_t data_size;
	int ret;

	// Read config
	ret = wmr_read_config_raw(wh, &data, &data_size);
	if (ret < 0)
		return ret;

	/* De-obfuscate the JSON config */
	/* FIXME: The header contains little-endian values that need swapping for big-endian */
	struct wmr_config_header *hdr = (struct wmr_config_header *)data;

	/* Take a copy of the header */
	memcpy(&wh->config_hdr, hdr, sizeof(struct wmr_config_header));

	WMR_INFO(wh, "Manufacturer: %.*s", (int)sizeof(hdr->manufacturer), hdr->manufacturer);
	WMR_INFO(wh, "Device: %.*s", (int)sizeof(hdr->device), hdr->device);
	WMR_INFO(wh, "Serial: %.*s", (int)sizeof(hdr->serial), hdr->serial);
	WMR_INFO(wh, "UID: %.*s", (int)sizeof(hdr->uid), hdr->uid);
	WMR_INFO(wh, "Name: %.*s", (int)sizeof(hdr->name), hdr->name);
	WMR_INFO(wh, "Revision: %.*s", (int)sizeof(hdr->revision), hdr->revision);
	WMR_INFO(wh, "Revision Date: %.*s", (int)sizeof(hdr->revision_date), hdr->revision_date);

	snprintf(wh->base.str, XRT_DEVICE_NAME_LEN, "%.*s", (int)sizeof(hdr->name), hdr->name);

	if (hdr->json_start >= data_size || (data_size - hdr->json_start) < hdr->json_size) {
		WMR_ERROR(wh, "Invalid WMR config block - incorrect sizes");
		free(data);
		return -1;
	}

	config_json_block = data + hdr->json_start + sizeof(uint16_t);
	for (unsigned int i = 0; i < hdr->json_size - sizeof(uint16_t); i++) {
		config_json_block[i] ^= wmr_config_key[i % sizeof(wmr_config_key)];
	}

	WMR_DEBUG(wh, "JSON config:\n%s", config_json_block);

	if (!wmr_hmd_config_parse(&wh->config, (char *)config_json_block, wh->log_level)) {
		free(data);
		return -1;
	}

	free(data);
	return 0;
}

/*
 *
 * Device members.
 *
 */

static xrt_result_t
wmr_hmd_get_3dof_tracked_pose(struct xrt_device *xdev,
                              enum xrt_input_name name,
                              uint64_t at_timestamp_ns,
                              struct xrt_space_relation *out_relation)
{
	DRV_TRACE_MARKER();

	struct wmr_hmd *wh = wmr_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&wh->base, wh->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	// Variables needed for prediction.
	uint64_t last_imu_timestamp_ns = 0;
	struct xrt_space_relation relation = {0};
	relation.relation_flags = XRT_SPACE_RELATION_BITMASK_ALL;
	relation.pose.position = wh->pose.position;
	relation.linear_velocity = (struct xrt_vec3){0, 0, 0};

	// Get data while holding the lock.
	os_mutex_lock(&wh->fusion.mutex);
	relation.pose.orientation = wh->fusion.i3dof.rot;
	relation.angular_velocity = wh->fusion.last_angular_velocity;
	last_imu_timestamp_ns = wh->fusion.last_imu_timestamp_ns;
	os_mutex_unlock(&wh->fusion.mutex);

	// No prediction needed.
	if (at_timestamp_ns < last_imu_timestamp_ns) {
		*out_relation = relation;
		return XRT_SUCCESS;
	}

	uint64_t prediction_ns = at_timestamp_ns - last_imu_timestamp_ns;
	double prediction_s = time_ns_to_s(prediction_ns);

	m_predict_relation(&relation, prediction_s, out_relation);
	wh->pose = out_relation->pose;

	return XRT_SUCCESS;
}

//! Specific pose corrections for Basalt and a WMR headset
XRT_MAYBE_UNUSED static inline struct xrt_pose
wmr_hmd_correct_pose_from_basalt(struct xrt_pose pose)
{
	struct xrt_quat q = {0.70710678, 0, 0, 0.70710678};
	math_quat_rotate(&q, &pose.orientation, &pose.orientation);
	math_quat_rotate_vec3(&q, &pose.position, &pose.position);

	// Correct swapped axes
	pose.position.y = -pose.position.y;
	pose.position.z = -pose.position.z;
	pose.orientation.y = -pose.orientation.y;
	pose.orientation.z = -pose.orientation.z;
	return pose;
}

//! Same axis correction as wmr_hmd_correct_pose_from_basalt(), for a free vector (e.g. angular
//! velocity) instead of a pose. This is a coordinate-frame relabeling (Basalt's axes -> WMR's),
//! not a physical rigid-body transform, so it applies to any vector quantity the same way it
//! applies to position: rotate by the fixed 90-degree quat, then negate the swapped y/z axes.
//! Unlike wmr_hmd_correct_pose_from_basalt() this is only reached under
//! WMR_FORWARD_ANGULAR_VELOCITY (see wmr_hmd_get_slam_tracked_pose()), so it is not applied by
//! default and has not been exercised against real Basalt output.
XRT_MAYBE_UNUSED static inline struct xrt_vec3
wmr_hmd_correct_vec3_from_basalt(struct xrt_vec3 v)
{
	struct xrt_quat q = {0.70710678, 0, 0, 0.70710678};
	math_quat_rotate_vec3(&q, &v, &v);
	v.y = -v.y;
	v.z = -v.z;
	return v;
}

static void
wmr_hmd_get_slam_tracked_pose(struct xrt_device *xdev,
                              enum xrt_input_name name,
                              uint64_t at_timestamp_ns,
                              struct xrt_space_relation *out_relation)
{
	DRV_TRACE_MARKER();

	struct wmr_hmd *wh = wmr_hmd(xdev);
	xrt_tracked_slam_get_tracked_pose(wh->tracking.slam, at_timestamp_ns, out_relation);

	int pose_bits = XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	bool pose_tracked = out_relation->relation_flags & pose_bits;

	// WMR_FORWARD_ANGULAR_VELOCITY (default off): capture the SLAM tracker's own angular
	// velocity for THIS at_timestamp_ns and whether the tracker itself considered it valid,
	// before relation_flags below is unconditionally clobbered to only ORIENTATION/POSITION.
	// This is the tracker's already-predicted velocity (predict_pose() in t_tracker_slam.cpp
	// with SLAM_PREDICTION_TYPE >= GYRO computes it from live gyro data, or dead-reckoning
	// integrates it), matching the orientation already returned for this timestamp -- NOT
	// wh->fusion.last_angular_velocity, which is the 3dof path's raw IMU fusion value as of its
	// own last_imu_timestamp_ns, an earlier and SLAM-uncorrected instant. Gate on the tracker's
	// own valid bit rather than assuming a populated field means real data: predict_pose() only
	// computes this vector for SLAM_PRED_GYRO and above and never sets the valid bit itself --
	// it inherits whatever the raw VIT-reported relation had, which for some VIT systems may not
	// include it at all.
	bool forward_angular_velocity =
	    debug_get_bool_option_wmr_forward_angular_velocity() &&
	    (out_relation->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0;
	struct xrt_vec3 angular_velocity = out_relation->angular_velocity;

	if (pose_tracked) {
#ifdef XRT_FEATURE_SLAM
		// !todo Correct pose depending on the VIT system in use, this should be done in the system itself.
		// For now, assume that we are using Basalt.
		wh->pose = wmr_hmd_correct_pose_from_basalt(out_relation->pose);
		if (forward_angular_velocity) {
			angular_velocity = wmr_hmd_correct_vec3_from_basalt(angular_velocity);
		}
#else
		wh->pose = out_relation->pose;
#endif
	} else {
		// No fresh SLAM pose this call -- don't forward a velocity paired with a pose we
		// are not updating either.
		forward_angular_velocity = false;
	}

	// wh->tracking.imu2me only re-anchors the pose to a different point rigidly attached to the
	// same physical head; a rigid body's angular velocity is the same at every point on it, so
	// unlike position, angular_velocity needs no further correction for this step.
	if (wh->tracking.imu2me) {
		math_pose_transform(&wh->pose, &wh->config.sensors.transforms.P_imu_me, &wh->pose);
	}

	out_relation->pose = wh->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	if (forward_angular_velocity) {
		out_relation->angular_velocity = angular_velocity;
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    out_relation->relation_flags | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	}
}

static xrt_result_t
wmr_hmd_get_tracked_pose(struct xrt_device *xdev,
                         enum xrt_input_name name,
                         int64_t at_timestamp_ns,
                         struct xrt_space_relation *out_relation)
{
	DRV_TRACE_MARKER();

	struct wmr_hmd *wh = wmr_hmd(xdev);

	at_timestamp_ns += (int64_t)(wh->tracked_offset_ms.val * (double)U_TIME_1MS_IN_NS);

	xrt_result_t xret = XRT_SUCCESS;
	if (wh->tracking.slam_enabled && wh->slam_over_3dof) {
		wmr_hmd_get_slam_tracked_pose(xdev, name, at_timestamp_ns, out_relation);
	} else {
		xret = wmr_hmd_get_3dof_tracked_pose(xdev, name, at_timestamp_ns, out_relation);
	}

	if (xret == XRT_SUCCESS) {
		math_pose_transform(&wh->offset, &out_relation->pose, &out_relation->pose);
	}

	return xret;
}

static void
wmr_hmd_destroy(struct xrt_device *xdev)
{
	DRV_TRACE_MARKER();

	struct wmr_hmd *wh = wmr_hmd(xdev);

	// Destroy the thread object.
	os_thread_helper_destroy(&wh->oth);

	// Disconnect tunnelled controllers
	os_mutex_lock(&wh->controller_status_lock);
	if (wh->controller[0] != NULL) {
		struct wmr_controller_connection *wcc = (struct wmr_controller_connection *)wh->controller[0];
		wmr_controller_connection_disconnect(wcc);
	}

	if (wh->controller[1] != NULL) {
		struct wmr_controller_connection *wcc = (struct wmr_controller_connection *)wh->controller[1];
		wmr_controller_connection_disconnect(wcc);
	}
	os_mutex_unlock(&wh->controller_status_lock);

	os_mutex_destroy(&wh->controller_status_lock);
	os_cond_destroy(&wh->controller_status_cond);

	if (wh->hid_hololens_sensors_dev != NULL) {
		os_hid_destroy(wh->hid_hololens_sensors_dev);
		wh->hid_hololens_sensors_dev = NULL;
	}

	if (wh->hid_control_dev != NULL) {
		/* Do any deinit if we have a deinit function */
		if (wh->hmd_desc && wh->hmd_desc->deinit_func) {
			wh->hmd_desc->deinit_func(wh);
		}
		os_hid_destroy(wh->hid_control_dev);
		wh->hid_control_dev = NULL;
	}

	// Destroy SLAM source and tracker
	xrt_frame_context_destroy_nodes(&wh->tracking.xfctx);

	// Destroy the fusion.
	m_imu_3dof_close(&wh->fusion.i3dof);

	os_mutex_destroy(&wh->fusion.mutex);
	os_mutex_destroy(&wh->hid_lock);

	u_device_free(&wh->base);
}

XRT_MAYBE_UNUSED static struct t_camera_calibration
wmr_hmd_get_cam_calib(struct wmr_hmd *wh, int cam_index)
{
	struct t_camera_calibration res;
	struct wmr_camera_config *wcalib = wh->config.tcams[cam_index];
	struct wmr_distortion_6KT *intr = &wcalib->distortion6KT;

	res.image_size_pixels.h = wcalib->roi.extent.h;
	res.image_size_pixels.w = wcalib->roi.extent.w;
	res.intrinsics[0][0] = intr->params.fx * (double)wcalib->roi.extent.w;
	res.intrinsics[1][1] = intr->params.fy * (double)wcalib->roi.extent.h;
	res.intrinsics[0][2] = intr->params.cx * (double)wcalib->roi.extent.w;
	res.intrinsics[1][2] = intr->params.cy * (double)wcalib->roi.extent.h;
	res.intrinsics[2][2] = 1.0;

	res.distortion_model = T_DISTORTION_WMR;
	res.wmr.k1 = intr->params.k[0];
	res.wmr.k2 = intr->params.k[1];
	res.wmr.p1 = intr->params.p1;
	res.wmr.p2 = intr->params.p2;
	res.wmr.k3 = intr->params.k[2];
	res.wmr.k4 = intr->params.k[3];
	res.wmr.k5 = intr->params.k[4];
	res.wmr.k6 = intr->params.k[5];
	res.wmr.codx = intr->params.dist_x;
	res.wmr.cody = intr->params.dist_y;
	res.wmr.rpmax = intr->params.metric_radius;

	return res;
}

XRT_MAYBE_UNUSED static struct xrt_vec2
wmr_hmd_camera_project(struct wmr_hmd *wh, struct xrt_vec3 p3d)
{
	float w = wh->config.cams[0].roi.extent.w;
	float h = wh->config.cams[0].roi.extent.h;
	float fx = wh->config.cams[0].distortion6KT.params.fx * w;
	float fy = wh->config.cams[0].distortion6KT.params.fy * h;
	float cx = wh->config.cams[0].distortion6KT.params.cx * w;
	float cy = wh->config.cams[0].distortion6KT.params.cy * h;
	float k1 = wh->config.cams[0].distortion6KT.params.k[0];
	float k2 = wh->config.cams[0].distortion6KT.params.k[1];
	float p1 = wh->config.cams[0].distortion6KT.params.p1;
	float p2 = wh->config.cams[0].distortion6KT.params.p2;
	float k3 = wh->config.cams[0].distortion6KT.params.k[2];
	float k4 = wh->config.cams[0].distortion6KT.params.k[3];
	float k5 = wh->config.cams[0].distortion6KT.params.k[4];
	float k6 = wh->config.cams[0].distortion6KT.params.k[5];

	float x = p3d.x;
	float y = p3d.y;
	float z = p3d.z;

	float xp = x / z;
	float yp = y / z;
	float rp2 = xp * xp + yp * yp;
	float cdist = (1 + rp2 * (k1 + rp2 * (k2 + rp2 * k3))) / (1 + rp2 * (k4 + rp2 * (k5 + rp2 * k6)));
#if 0 // OpenCV model
	float deltaX = 2 * p1 * xp * yp + p2 * (rp2 + 2 * xp * xp);
	float deltaY = 2 * p2 * xp * yp + p1 * (rp2 + 2 * yp * yp);
#else // Azure Kinect model (see comment in wmr_hmd_create_stereo_camera_calib)
	float deltaX = p1 * xp * yp + p2 * (rp2 + 2 * xp * xp);
	float deltaY = p2 * xp * yp + p1 * (rp2 + 2 * yp * yp);
#endif
	float xpp = xp * cdist + deltaX;
	float ypp = yp * cdist + deltaY;
	float u = fx * xpp + cx;
	float v = fy * ypp + cy;

	struct xrt_vec2 p2d = {u, v};
	return p2d;
}


/*!
 * Creates an OpenCV-compatible @ref t_stereo_camera_calibration pointer from
 * the WMR config.
 *
 * Note that the camera model used on WMR headsets seems to be the same as the
 * one in Azure-Kinect-Sensor-SDK. That model is slightly different than
 * OpenCV's in the following ways:
 * 1. There are "center of distortion", codx and cody, parameters
 * 2. The terms that use the tangential parameters, p1 and p2, aren't multiplied by 2
 * 3. There is a "metric radius" that delimits a valid area of distortion/undistortion
 *
 * Thankfully, parameters of points 1 and 2 tend to be almost zero in practice. For 3, we place metric_radius into
 * the calibration struct so that downstream tracking algorithms can use it as needed.
 */
XRT_MAYBE_UNUSED static struct t_stereo_camera_calibration *
wmr_hmd_create_stereo_camera_calib(struct wmr_hmd *wh)
{
	struct t_stereo_camera_calibration *calib = NULL;
	t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_WMR);


	// Intrinsics
	for (int i = 0; i < 2; i++) {
		calib->view[i] = wmr_hmd_get_cam_calib(wh, i);
	}

	// Extrinsics

	// Compute transform from HT1 to HT0 (HT0 space into HT1 space)
	struct wmr_camera_config *ht1 = &wh->config.cams[1];
	calib->camera_translation[0] = ht1->translation.x;
	calib->camera_translation[1] = ht1->translation.y;
	calib->camera_translation[2] = ht1->translation.z;
	calib->camera_rotation[0][0] = ht1->rotation.v[0];
	calib->camera_rotation[0][1] = ht1->rotation.v[1];
	calib->camera_rotation[0][2] = ht1->rotation.v[2];
	calib->camera_rotation[1][0] = ht1->rotation.v[3];
	calib->camera_rotation[1][1] = ht1->rotation.v[4];
	calib->camera_rotation[1][2] = ht1->rotation.v[5];
	calib->camera_rotation[2][0] = ht1->rotation.v[6];
	calib->camera_rotation[2][1] = ht1->rotation.v[7];
	calib->camera_rotation[2][2] = ht1->rotation.v[8];

	return calib;
}

//! Extended camera calibration info for SLAM
XRT_MAYBE_UNUSED static void
wmr_hmd_fill_slam_cams_calibration(struct wmr_hmd *wh)
{
	wh->tracking.slam_calib.cam_count = wh->config.tcam_count;

	// Fill camera 0
	struct xrt_pose P_imu_c0 = wh->config.sensors.accel.pose;
	struct xrt_matrix_4x4 T_imu_c0;
	math_matrix_4x4_isometry_from_pose(&P_imu_c0, &T_imu_c0);
	wh->tracking.slam_calib.cams[0] = (struct t_slam_camera_calibration){
	    .base = wmr_hmd_get_cam_calib(wh, 0),
	    .T_imu_cam = T_imu_c0,
	    .frequency = CAMERA_FREQUENCY,
	};

	// Fill remaining cameras
	for (int i = 1; i < wh->config.tcam_count; i++) {
		struct xrt_pose P_ci_c0 = wh->config.tcams[i]->pose;

		if (i == 2 || i == 3) {
			//! @note The calibration json for the reverb G2v2 (the only 4-camera wmr
			//! headset we know about) has the HT2 and HT3 extrinsics flipped compared
			//! to the order the third and fourth camera images come from usb.
			P_ci_c0 = wh->config.tcams[i == 2 ? 3 : 2]->pose;
		}

		struct xrt_pose P_c0_ci;
		math_pose_invert(&P_ci_c0, &P_c0_ci);

		struct xrt_pose P_imu_ci;
		math_pose_transform(&P_imu_c0, &P_c0_ci, &P_imu_ci);

		struct xrt_matrix_4x4 T_imu_ci;
		math_matrix_4x4_isometry_from_pose(&P_imu_ci, &T_imu_ci);

		wh->tracking.slam_calib.cams[i] = (struct t_slam_camera_calibration){
		    .base = wmr_hmd_get_cam_calib(wh, i),
		    .T_imu_cam = T_imu_ci,
		    .frequency = CAMERA_FREQUENCY,
		};
	}
}

/*!
 * Where the constellation tracker asks "where was the headset at time T", so it can place the
 * head-mounted cameras in the world before solving for a controller.
 *
 * Without this the mosaic has no tracking origin and CameraMosaic::getTrackingOriginPose returns
 * XRT_POSE_IDENTITY -- the tracker then assumes the headset never moves and never turns, so every
 * controller position comes out in a frame bolted to the IMU's initial orientation instead of the
 * world. Measured 2026-08-12: controllers tracked and displaced correctly in all three axes, but
 * along visibly wrong axes, and turning the head did not carry them.
 *
 * Two details this has to get right:
 *
 * - The tracker discards the observation unless BOTH position and orientation are valid
 *   (getTrackingOriginPose -> std::nullopt). With a 3dof head there is no valid position, but in
 *   that mode the headset IS the fixed origin, so (0,0,0) is the honest answer rather than a
 *   missing one -- and orientation, the part that was missing entirely, is real either way.
 * - Orientation is NOT forced: if the fusion has not produced a valid one yet, an identity
 *   quaternion would silently claim the head is facing forward. Leave the flags clear and let the
 *   tracker skip the frame.
 */
static void
wmr_hmd_constellation_tracking_source_get_tracked_pose(struct t_constellation_tracker_tracking_source *tracking_source,
                                                       int64_t when_ns,
                                                       struct xrt_space_relation *out_relation)
{
	struct wmr_hmd *wh = container_of(tracking_source, struct wmr_hmd, tracking.constellation_tracking_source);

	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;

	xrt_result_t xret = wmr_hmd_get_tracked_pose(&wh->base, XRT_INPUT_GENERIC_HEAD_POSE, when_ns, out_relation);
	if (xret != XRT_SUCCESS ||
	    (out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return;
	}

	if ((out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0) {
		out_relation->pose.position = (struct xrt_vec3){0.f, 0.f, 0.f};
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    out_relation->relation_flags | XRT_SPACE_RELATION_POSITION_VALID_BIT);
	}

	// Last line of defence: this pose becomes the origin of every camera in the mosaic, so a single
	// non-finite value here turns every controller position the tracker produces into NaN. Reported
	// as invalid instead, which the tracker already handles by skipping the observation.
	// NOT math_pose_validate(): its quaternion check compares the norm against 1 with a tolerance of
	// FLOAT_EPSILON (~1.2e-7), which a perfectly usable pose fails after filtering and prediction
	// accumulate a little numerical error. Measured 2026-08-12: it rejected 5528 consecutive SLAM
	// head poses in 40 s -- every single one -- and with no valid origin the tracker skipped every
	// frame, so the controllers went from noisy to nothing. math_quat_validate_within_1_percent is
	// the in-tree validator meant for this; combined with the position check it still catches the
	// NaN this guard exists for.
	if (!math_vec3_validate(&out_relation->pose.position) ||
	    !math_quat_validate_within_1_percent(&out_relation->pose.orientation)) {
		static int reject_log_count = 0;
		if ((reject_log_count++ % 200) == 0) {
			WMR_WARN(wh,
			         "Head pose for the constellation tracker is not usable (#%d): pos=(%f, %f, %f) "
			         "quat=(%f, %f, %f, %f) norm=%f flags=0x%x xret=%d",
			         reject_log_count, out_relation->pose.position.x, out_relation->pose.position.y,
			         out_relation->pose.position.z, out_relation->pose.orientation.x,
			         out_relation->pose.orientation.y, out_relation->pose.orientation.z,
			         out_relation->pose.orientation.w,
			         sqrt((double)out_relation->pose.orientation.x * out_relation->pose.orientation.x +
			              (double)out_relation->pose.orientation.y * out_relation->pose.orientation.y +
			              (double)out_relation->pose.orientation.z * out_relation->pose.orientation.z +
			              (double)out_relation->pose.orientation.w * out_relation->pose.orientation.w),
			         (unsigned int)out_relation->relation_flags, (int)xret);
		}
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	}
}

/*!
 * Builds a constellation-tracker camera mosaic from the same per-camera intrinsics/extrinsics
 * SLAM already uses (@ref wmr_hmd_fill_slam_cams_calibration), and creates the tracker. Indexed
 * in raw USB frame order (0..tcam_count-1), matching @ref wmr_camera_open_config.ctrl_cam_sinks
 * -- NOT the calibration JSON's HT-numbering, hence the same camera-2/3 pose swap SLAM applies
 * for the same reason (see the comment there).
 *
 * No device is registered with the tracker here (that needs the per-controller LED model, a
 * later patch) -- this only wires cameras in, so the tracker has somewhere to send unmatched
 * blob observations once something calls @ref t_constellation_tracker_add_device.
 *
 * Sets wh->tracking.constellation_tracker and wh->tracking.constellation_cam_blob_sinks[] on
 * success; leaves both zeroed on failure (logged, not fatal -- controller tracking stays
 * orientation-only exactly as if WMR_CONSTELLATION_CONTROLLERS were never set).
 */
static void
wmr_hmd_create_constellation_tracker(struct wmr_hmd *wh)
{
	struct t_constellation_tracker_params params = {
	    .flags = T_CONSTELLATION_TRACKER_FLAGS_NONE,
	    .num_mosaics = 1,
	};
	struct t_constellation_tracker_camera_mosaic *mosaic = &params.mosaics[0];

	// The cameras below are given IMU-relative poses; this is what the tracker composes them with
	// to get where they actually are in the world each frame.
	wh->tracking.constellation_tracking_source.get_tracked_pose =
	    wmr_hmd_constellation_tracking_source_get_tracked_pose;
	mosaic->tracking_origin = &wh->tracking.constellation_tracking_source;

	for (int i = 0; i < wh->config.tcam_count && i < XRT_TRACKING_MAX_CAMS; i++) {
		struct xrt_pose P_imu_ci;

		if (i == 0) {
			P_imu_ci = wh->config.sensors.accel.pose;
		} else {
			struct xrt_pose P_ci_c0 = wh->config.tcams[i]->pose;

			//! @note Same HT2/HT3 extrinsics-vs-USB-order swap as wmr_hmd_fill_slam_cams_calibration.
			if (i == 2 || i == 3) {
				P_ci_c0 = wh->config.tcams[i == 2 ? 3 : 2]->pose;
			}

			struct xrt_pose P_c0_ci;
			math_pose_invert(&P_ci_c0, &P_c0_ci);
			math_pose_transform(&wh->config.sensors.accel.pose, &P_c0_ci, &P_imu_ci);
		}

		mosaic->cameras[mosaic->num_cameras++] = (struct t_constellation_tracker_camera){
		    .calibration = wmr_hmd_get_cam_calib(wh, i),
		    .pose_in_origin = P_imu_ci,
		    .has_concrete_pose = true, // Real extrinsics from the headset's own factory calibration.
		};
	}

	int ret = t_constellation_tracker_create(&wh->tracking.xfctx, &params, &wh->tracking.constellation_tracker);
	if (ret != 0) {
		WMR_WARN(wh, "Failed to create constellation tracker for controllers, code %d -- controllers stay "
		             "orientation-only",
		         ret);
		wh->tracking.constellation_tracker = NULL;
		return;
	}

	for (size_t i = 0; i < mosaic->num_cameras; i++) {
		wh->tracking.constellation_cam_blob_sinks[i] = mosaic->cameras[i].blob_sink;
	}

	WMR_INFO(wh, "Constellation tracker created for controller positional tracking (%zu cameras, no devices "
	             "registered yet)",
	         mosaic->num_cameras);
}

struct t_constellation_tracker *
wmr_hmd_get_constellation_tracker(struct xrt_device *head)
{
	struct wmr_hmd *wh = (struct wmr_hmd *)head;
	return wh->tracking.constellation_tracker;
}

XRT_MAYBE_UNUSED static struct t_imu_calibration
wmr_hmd_get_imu_calib(struct wmr_hmd *wh)
{
	float *at = wh->config.sensors.accel.mix_matrix.v;
	struct xrt_vec3 ao = wh->config.sensors.accel.bias_offsets;
	struct xrt_vec3 ab = wh->config.sensors.accel.bias_var;
	struct xrt_vec3 an = wh->config.sensors.accel.noise_std;

	float *gt = wh->config.sensors.gyro.mix_matrix.v;
	struct xrt_vec3 go = wh->config.sensors.gyro.bias_offsets;
	struct xrt_vec3 gb = wh->config.sensors.gyro.bias_var;
	struct xrt_vec3 gn = wh->config.sensors.gyro.noise_std;

	struct t_imu_calibration calib = {
	    .accel =
	        {
	            .transform = {{at[0], at[1], at[2]}, {at[3], at[4], at[5]}, {at[6], at[7], at[8]}},
	            .offset = {-ao.x, -ao.y, -ao.z}, // negative because slam system will add, not subtract
	            .bias_std = {sqrtf(ab.x), sqrtf(ab.y), sqrtf(ab.z)}, // sqrt because we want stdev not variance
	            .noise_std = {an.x, an.y, an.z},
	        },
	    .gyro =
	        {
	            .transform = {{gt[0], gt[1], gt[2]}, {gt[3], gt[4], gt[5]}, {gt[6], gt[7], gt[8]}},
	            .offset = {-go.x, -go.y, -go.z},
	            .bias_std = {sqrtf(gb.x), sqrtf(gb.y), sqrtf(gb.z)},
	            .noise_std = {gn.x, gn.y, gn.z},
	        },
	};

	// T204 blind spot: nothing in this project had ever printed the factory IMU calibration
	// actually in use (Basalt's print_calibration() is dead code), yet a wrong/degenerate
	// gyro mix_matrix would manifest EXACTLY as the measured motion-proportional roll drift
	// (+0.9 deg/min worn vs ~0 at rest) and the yaw->tilt coupling. One INFO block per boot.
	WMR_INFO(wh, "IMU factory calib: gyro mix [%f %f %f; %f %f %f; %f %f %f] bias [%f %f %f]", //
	         gt[0], gt[1], gt[2], gt[3], gt[4], gt[5], gt[6], gt[7], gt[8], go.x, go.y, go.z);
	WMR_INFO(wh, "IMU factory calib: accel mix [%f %f %f; %f %f %f; %f %f %f] bias [%f %f %f]", //
	         at[0], at[1], at[2], at[3], at[4], at[5], at[6], at[7], at[8], ao.x, ao.y, ao.z);

	if (debug_get_bool_option_wmr_hmd_gyro_mount_fix()) {
		// WMR_HMD_GYRO_MOUNT_FIX -- T211 gyro mounting-misalignment correction.
		//
		// T211 (chair-oscillation test, headset static on a rotating chair) fit the
		// gravity-projection tilt angles (t211's own convention: axis0=atan2(g_body.x,
		// g_body.y), axis2=atan2(g_body.z,g_body.y), body-Y up, g_body = world-up expressed
		// in the current body frame) against cumulative yaw and found a real, motion-
		// proportional, non-decaying leak: axis0 = +0.09116*cum_yaw (R2=0.60), axis2 =
		// -0.08948*cum_yaw (R2=0.44). This is NOT explained by the factory gyro mix_matrix
		// (near-identity, off-diagonals <=0.0016, see the INFO line above) -- it's a real
		// mechanical rotation of the gyro die relative to the true head-up axis that the
		// factory linear-gain/orthogonality calibration doesn't model at all.
		//
		// Getting from those two fit coefficients to the actual misaligned axis is NOT the
		// naive "read them off as the (x,z) components of the tilt" -- that reading has the
		// axes backwards. The exact kinematics of the gravity-projection measure (from
		// dR/dt = R[omega_body]_x, verified here by direct zero-order numerical
		// differentiation from identity orientation, and confirmed on T211's own real data
		// via a small-tilt-restricted instantaneous regression) give, at zero tilt:
		//   d(axis0)/dt = omega_body.z         d(axis2)/dt = -omega_body.x
		// i.e. axis0's coefficient is the misaligned axis's Z-component and axis2's
		// coefficient is MINUS its X-component -- so:
		//   Vx = -(axis2 coeff) = +0.08948      Vz = +(axis0 coeff) = +0.09116
		// (same sign on both, not opposite -- the small-tilt-restricted regression on the
		// real T211 data independently confirms Vx and Vz are both positive and comparable
		// in size, contradicting the naive same-axis/opposite-sign reading). The magnitude
		// is unaffected by the axis correction: |leak| = sqrt(Vx^2+Vz^2) = 0.12774 ->
		// misalignment angle = asin(0.12774) = 7.339 deg.
		//
		// R_fix is the minimal rotation taking the misaligned axis V=(Vx,Vy,Vz), Vy =
		// sqrt(1-Vx^2-Vz^2), back onto true up (0,1,0): axis = normalize(V x (0,1,0)) =
		// normalize((-Vz,0,Vx)) = (-0.71365, 0, 0.70050), angle = acos(Vy) = 7.339 deg.
		// Baked below via the standard Rodrigues formula, row-major, verified orthogonal
		// (det=1, R^T R = I) and R_fix @ V = (0,1,0) to double-precision epsilon.
		//
		// Composed as R_fix @ existing_transform (left-multiply: applied AFTER the factory
		// mix, on already-factory-calibrated samples) so a near-identity factory matrix
		// leaves this close to a pure R_fix, and any real factory correction underneath is
		// preserved rather than overwritten.
		//
		// Gyro only, deliberately -- the leak evidence is entirely gyro-axis (it persists
		// motionless and tracks yaw rate specifically). Per the 0064/0065 lesson (controller
		// left-hand reflection: extending an unverified frame fix to the accelerometer and
		// having to revert it after measuring worse precession), do NOT co-flip the accel
		// transform on gyro-only evidence -- accel touches gravity anchoring and has its own,
		// separate error surface; it gets no fix here without its own measurement.
		//
		// Off by default. Validation plan (not yet done): repeat the T211 chair-oscillation
		// test with this env var set; a working fix should collapse the axis0/axis2 vs.
		// cum_yaw leak fit toward ~0 slope, and should reduce the worn-gameplay roll drift
		// (T203: +0.84 to +0.93 deg/min) toward the motionless-rest baseline (~-0.03 deg/min).
		static const double gyro_mount_fix[3][3] = {
		    {0.9959801998, -0.0894800000, -0.0040952726},
		    {0.0894800000, 0.9918080379, 0.0911600000},
		    {-0.0040952726, -0.0911600000, 0.9958278381},
		};

		double composed[3][3];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				double sum = 0.0;
				for (int k = 0; k < 3; k++) {
					sum += gyro_mount_fix[i][k] * calib.gyro.transform[k][j];
				}
				composed[i][j] = sum;
			}
		}
		memcpy(calib.gyro.transform, composed, sizeof(composed));

		WMR_INFO(wh,
		         "WMR_HMD_GYRO_MOUNT_FIX active: composed gyro transform "
		         "[%f %f %f; %f %f %f; %f %f %f]",
		         calib.gyro.transform[0][0], calib.gyro.transform[0][1], calib.gyro.transform[0][2],
		         calib.gyro.transform[1][0], calib.gyro.transform[1][1], calib.gyro.transform[1][2],
		         calib.gyro.transform[2][0], calib.gyro.transform[2][1], calib.gyro.transform[2][2]);
	}

	return calib;
}

//! Extended IMU calibration data for SLAM
XRT_MAYBE_UNUSED static void
wmr_hmd_fill_slam_imu_calibration(struct wmr_hmd *wh)
{
	//! @note `average_imus` might change during runtime but the calibration data will be already submitted
	double imu_frequency = wh->average_imus ? IMU_FREQUENCY / IMU_SAMPLES_PER_PACKET : IMU_FREQUENCY;

	struct t_slam_imu_calibration imu_calib = {
	    .base = wmr_hmd_get_imu_calib(wh),
	    .frequency = imu_frequency,
	};

	wh->tracking.slam_calib.imu = imu_calib;
}

XRT_MAYBE_UNUSED static void
wmr_hmd_fill_slam_calibration(struct wmr_hmd *wh)
{
	wmr_hmd_fill_slam_imu_calibration(wh);
	wmr_hmd_fill_slam_cams_calibration(wh);
}

static void
wmr_hmd_switch_hmd_tracker(void *wh_ptr)
{
	DRV_TRACE_MARKER();

	struct wmr_hmd *wh = (struct wmr_hmd *)wh_ptr;
	wh->slam_over_3dof = !wh->slam_over_3dof;
	struct u_var_button *btn = &wh->gui.switch_tracker_btn;

	if (wh->slam_over_3dof) { // Use SLAM
		snprintf(btn->label, sizeof(btn->label), "Switch to 3DoF Tracking");
	} else { // Use 3DoF
		snprintf(btn->label, sizeof(btn->label), "Switch to SLAM Tracking");
		os_mutex_lock(&wh->fusion.mutex);
		m_imu_3dof_reset(&wh->fusion.i3dof);
		wh->fusion.i3dof.rot = wh->pose.orientation;
		os_mutex_unlock(&wh->fusion.mutex);
	}
}

static struct xrt_slam_sinks *
wmr_hmd_slam_track(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();

	struct xrt_slam_sinks *sinks = NULL;

#ifdef XRT_FEATURE_SLAM
	struct t_slam_tracker_config config = {0};
	t_slam_fill_default_config(&config);
	config.cam_count = wh->config.slam_cam_count;
	wh->tracking.slam_calib.cam_count = wh->config.slam_cam_count;
	config.slam_calib = &wh->tracking.slam_calib;
	if (debug_get_option_slam_submit_from_start() == NULL) {
		config.submit_from_start = true;
	}

	int create_status = t_slam_create(&wh->tracking.xfctx, &config, &wh->tracking.slam, &sinks);
	if (create_status != 0) {
		return NULL;
	}

	int start_status = t_slam_start(wh->tracking.slam);
	if (start_status != 0) {
		return NULL;
	}

	WMR_DEBUG(wh, "WMR HMD SLAM tracker successfully started");
#endif

	return sinks;
}

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
static enum t_camera_orientation
wmr_hmd_guess_camera_orientation(struct wmr_hmd *wh)
{
	struct xrt_quat Q_ht0_me = wh->config.sensors.transforms.P_ht0_me.orientation;
	struct xrt_vec2 swing = {0};
	float twist = 0;
	math_quat_to_swing_twist(&Q_ht0_me, &swing, &twist);
	WMR_DEBUG(wh, "HT0 twist value is %f", twist);

	float abstwist = fabsf(twist);

	// Bottom quadrant
	if (abstwist < M_PI / 4) {
		WMR_DEBUG(wh, "I think this headset has CAMERA_ORIENTATION_0 front cameras!");
		return CAMERA_ORIENTATION_0;
	}

	// Top quadrant
	if (abstwist > 3 * M_PI / 4) {
		WMR_DEBUG(wh, "I think this headset has CAMERA_ORIENTATION_180 front cameras!");
		return CAMERA_ORIENTATION_180;
	}

	// Right quadrant
	if (twist < 0) {
		WMR_DEBUG(wh, "I think this headset has CAMERA_ORIENTATION_90 front cameras!");
		return CAMERA_ORIENTATION_90;
	}

	// Left quadrant
	WMR_DEBUG(wh, "I think this headset has CAMERA_ORIENTATION_270 front cameras!");
	return CAMERA_ORIENTATION_270;
}
#endif

static int
wmr_hmd_hand_track(struct wmr_hmd *wh,
                   struct t_stereo_camera_calibration *stereo_calib,
                   struct xrt_hand_masks_sink *masks_sink,
                   struct xrt_slam_sinks **out_sinks,
                   struct xrt_device **out_device)
{
	DRV_TRACE_MARKER();

	struct xrt_slam_sinks *sinks = NULL;
	struct xrt_device *device = NULL;

#ifdef XRT_BUILD_DRIVER_HANDTRACKING

	struct t_camera_extra_info extra_camera_info = {0};

	enum t_camera_orientation ori_guess = CAMERA_ORIENTATION_0;

	if (wh->hmd_desc->hmd_type == WMR_HEADSET_GENERIC || //
	    wh->hmd_desc->hmd_type == WMR_HEADSET_REVERB_G2) {
		ori_guess = wmr_hmd_guess_camera_orientation(wh);
	}

	for (int i = 0; i < 2; i++) {
		extra_camera_info.views[i].camera_orientation = ori_guess;
		extra_camera_info.views[i].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
		float w = wh->config.cams[i].roi.extent.w;
		float h = wh->config.cams[i].roi.extent.h;
		float cx = wh->config.cams[i].distortion6KT.params.cx * w;
		float cy = wh->config.cams[i].distortion6KT.params.cy * h;
		float rpmax = wh->config.cams[i].distortion6KT.params.metric_radius;
		struct xrt_vec3 p3d = {rpmax, 0, 1}; // Right-most border of the metric_radius circle in the Z=1 plane
		struct xrt_vec2 p2d = wmr_hmd_camera_project(wh, p3d);
		float radius = (p2d.x - cx) / w;
		extra_camera_info.views[i].boundary.circle.normalized_center = (struct xrt_vec2){cx / w, cy / h};
		extra_camera_info.views[i].boundary.circle.normalized_radius = radius;
	}

	struct t_hand_tracking_create_info create_info = {.cams_info = extra_camera_info, .masks_sink = masks_sink};

	int create_status = ht_device_create(&wh->tracking.xfctx, //
	                                     stereo_calib,        //
	                                     create_info,         //
	                                     &sinks,              //
	                                     &device);
	if (create_status != 0) {
		return create_status;
	}

	device = multi_create_tracking_override(XRT_TRACKING_OVERRIDE_ATTACHED, device, &wh->base,
	                                        XRT_INPUT_GENERIC_HEAD_POSE, &wh->config.sensors.transforms.P_ht0_me);

	WMR_DEBUG(wh, "WMR HMD hand tracker successfully created");
#endif

	*out_sinks = sinks;
	*out_device = device;

	return 0;
}

static void
wmr_hmd_setup_ui(struct wmr_hmd *wh)
{
	u_var_add_root(wh, "WMR HMD", true);

	u_var_add_gui_header(wh, NULL, "Tracking");
	if (wh->tracking.slam_enabled) {
		wh->gui.switch_tracker_btn.cb = wmr_hmd_switch_hmd_tracker;
		wh->gui.switch_tracker_btn.ptr = wh;
		u_var_add_button(wh, &wh->gui.switch_tracker_btn, "Switch to 3DoF Tracking");
	}
	u_var_add_pose(wh, &wh->pose, "Tracked Pose");
	u_var_add_pose(wh, &wh->offset, "Pose Offset");
	u_var_add_bool(wh, &wh->average_imus, "Average IMU samples");
	u_var_add_draggable_f32(wh, &wh->tracked_offset_ms, "Timecode offset(ms)");

	u_var_add_gui_header(wh, NULL, "3DoF Tracking");
	m_imu_3dof_add_vars(&wh->fusion.i3dof, wh, "");

	u_var_add_gui_header(wh, NULL, "SLAM Tracking");
	u_var_add_ro_text(wh, wh->gui.slam_status, "Tracker status");
	u_var_add_bool(wh, &wh->tracking.imu2me, "Correct IMU pose to middle of eyes");

	u_var_add_gui_header(wh, NULL, "Hand Tracking");
	u_var_add_ro_text(wh, wh->gui.hand_status, "Tracker status");

	u_var_add_gui_header(wh, NULL, "Hololens Sensors' Companion device");
	u_var_add_u8(wh, &wh->proximity_sensor, "HMD Proximity");
	u_var_add_u16(wh, &wh->raw_ipd, "HMD IPD");

	if (wh->hmd_desc->screen_enable_func) {
		// Enabling/disabling the HMD screen at runtime is supported. Add button to debug GUI.
		wh->gui.hmd_screen_enable_btn.cb = wmr_hmd_screen_enable_toggle;
		wh->gui.hmd_screen_enable_btn.ptr = wh;
		u_var_add_button(wh, &wh->gui.hmd_screen_enable_btn, "HMD Screen [On/Off]");
	}

	u_var_add_gui_header(wh, NULL, "Misc");
	u_var_add_log_level(wh, &wh->log_level, "log_level");
}

/*!
 * Procedure to setup trackers: 3dof, SLAM and hand tracking.
 *
 * Determines which trackers to initialize and starts them.
 * Fills @p out_sinks to stream raw data to for tracking.
 * In the case of hand tracking being enabled, it returns a hand tracker device in @p out_handtracker.
 *
 * @param wh the wmr headset device
 * @param out_sinks sinks to stream video/IMU data to for tracking
 * @param out_handtracker a newly created hand tracker device
 * @return true on success, false when an unexpected state is reached.
 */
static bool
wmr_hmd_setup_trackers(struct wmr_hmd *wh, struct xrt_slam_sinks *out_sinks, struct xrt_device **out_handtracker)
{
	// We always have at least 3dof HMD tracking
	bool dof3_enabled = true;

	// Without camera streaming there is no data for SLAM or hand tracking.
	bool cams_enabled = debug_get_bool_option_wmr_cameras();

	// Decide whether to initialize the SLAM tracker
	bool slam_wanted = debug_get_bool_option_wmr_slam();
#ifdef XRT_FEATURE_SLAM
	bool slam_supported = true;
#else
	bool slam_supported = false;
#endif
	bool slam_enabled = slam_supported && slam_wanted && cams_enabled;

	// Decide whether to initialize the hand tracker
	bool hand_wanted = debug_get_bool_option_wmr_handtracking();
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	bool hand_supported = true;
#else
	bool hand_supported = false;
#endif
	bool hand_enabled = hand_supported && hand_wanted && cams_enabled;

	wh->base.supported.orientation_tracking = dof3_enabled || slam_enabled;
	wh->base.supported.position_tracking = slam_enabled;
	wh->base.supported.hand_tracking = false; // out_handtracker will handle it

	wh->tracking.slam_enabled = slam_enabled;
	wh->tracking.hand_enabled = hand_enabled;
	wh->tracking.imu2me = true;

	wh->slam_over_3dof = slam_enabled; // We prefer SLAM over 3dof tracking if possible

	const char *slam_status = wh->tracking.slam_enabled ? "Enabled"
	                          : !slam_wanted            ? "Disabled by the user (envvar set to false)"
	                          : !cams_enabled           ? "Disabled by the user (WMR_CAMERAS=0)"
	                          : !slam_supported         ? "Unavailable (not built)"
	                                                    : NULL;

	const char *hand_status = wh->tracking.hand_enabled ? "Enabled"
	                          : !hand_wanted            ? "Disabled by the user (envvar set to false)"
	                          : !cams_enabled           ? "Disabled by the user (WMR_CAMERAS=0)"
	                          : !hand_supported         ? "Unavailable (not built)"
	                                                    : NULL;

	assert(slam_status != NULL && hand_status != NULL);

	(void)snprintf(wh->gui.slam_status, sizeof(wh->gui.slam_status), "%s", slam_status);
	(void)snprintf(wh->gui.hand_status, sizeof(wh->gui.hand_status), "%s", hand_status);

	struct t_stereo_camera_calibration *stereo_calib = wmr_hmd_create_stereo_camera_calib(wh);
	wmr_hmd_fill_slam_calibration(wh);

	// Initialize 3DoF tracker
	// See the same change in wmr_controller_base.c. The headset was not measured drifting the way
	// the controllers were, but it runs the same fusion off the same kind of sensor, and the
	// estimate only ever runs while the device is provably still.
	m_imu_3dof_init(&wh->fusion.i3dof, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_USE_GYRO_BIAS_AUTO);

	// Initialize SLAM tracker
	struct xrt_slam_sinks *slam_sinks = NULL;
	if (wh->tracking.slam_enabled) {
		slam_sinks = wmr_hmd_slam_track(wh);
		if (slam_sinks == NULL) {
			WMR_WARN(wh, "Unable to setup the SLAM tracker");
			return false;
		}
	}

	// Initialize hand tracker
	struct xrt_slam_sinks *hand_sinks = NULL;
	struct xrt_device *hand_device = NULL;
	struct xrt_hand_masks_sink *masks_sink = slam_sinks ? slam_sinks->hand_masks : NULL;
	if (wh->tracking.hand_enabled) {
		int hand_status = wmr_hmd_hand_track(wh, stereo_calib, masks_sink, &hand_sinks, &hand_device);
		if (hand_status != 0 || hand_sinks == NULL || hand_device == NULL) {
			WMR_WARN(wh, "Unable to setup the hand tracker");
			return false;
		}
	}

	t_stereo_camera_calibration_reference(&stereo_calib, NULL);

	// Setup sinks depending on tracking configuration
	struct xrt_slam_sinks entry_sinks = {0};
	if (slam_enabled && hand_enabled) {
		struct xrt_frame_sink *entry_cam0_sink = NULL;
		struct xrt_frame_sink *entry_cam1_sink = NULL;

		u_sink_split_create(&wh->tracking.xfctx, slam_sinks->cams[0], hand_sinks->cams[0], &entry_cam0_sink);
		u_sink_split_create(&wh->tracking.xfctx, slam_sinks->cams[1], hand_sinks->cams[1], &entry_cam1_sink);

		entry_sinks = *slam_sinks;
		entry_sinks.cams[0] = entry_cam0_sink;
		entry_sinks.cams[1] = entry_cam1_sink;
	} else if (slam_enabled) {
		entry_sinks = *slam_sinks;
	} else if (hand_enabled) {
		entry_sinks = *hand_sinks;
	} else {
		entry_sinks = (struct xrt_slam_sinks){0};
	}

	*out_sinks = entry_sinks;
	*out_handtracker = hand_device;
	return true;
}

static bool
wmr_hmd_request_controller_status(struct wmr_hmd *wh)
{
	DRV_TRACE_MARKER();
	unsigned char cmd[64] = {WMR_MS_HOLOLENS_MSG_BT_CONTROL, WMR_MS_HOLOLENS_MSG_CONTROLLER_STATUS};
	return wmr_hmd_send_controller_packet(wh, cmd, sizeof(cmd));
}

static xrt_result_t
compute_distortion_wmr(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct wmr_hmd *wh = wmr_hmd(xdev);

	u_compute_distortion_poly_3k(&wh->config.eye_params[view].poly_3k, view, u, v, out_result);

	return XRT_SUCCESS;
}

static xrt_result_t
get_compositor_info_wmr(struct xrt_device *xdev,
                        const struct xrt_device_compositor_mode *mode,
                        struct xrt_device_compositor_info *out_info)
{
	struct wmr_hmd *wh = wmr_hmd(xdev);

	double scanout_multiplier = 0.0;
	enum xrt_scanout_direction scanout_direction = XRT_SCANOUT_DIRECTION_NONE;

	if (wh->hmd_desc->hmd_type == WMR_HEADSET_SAMSUNG_800ZAA ||
	    wh->hmd_desc->hmd_type == WMR_HEADSET_SAMSUNG_XE700X3AI) {
		scanout_direction = XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM;
		scanout_multiplier = 1600.0 / 1624.0;
	}

	*out_info = (struct xrt_device_compositor_info){
	    .scanout_direction = scanout_direction,
	    .scanout_time_ns = (int64_t)(mode->frame_interval_ns * scanout_multiplier),
	};

	return XRT_SUCCESS;
}

/*!
 * Feeds @ref wmr_hmd::proximity_sensor (updated by @ref control_ipd_value_decode as
 * WMR_CONTROL_MSG_IPD_VALUE packets arrive) into the XRT_INPUT_GENERIC_HEAD_DETECT input
 * that Monado's state tracker reads for XR_EXT_user_presence -- see
 * GET_STATIC_XDEV_BY_ROLE()/XRT_INPUT_GENERIC_HEAD_DETECT handling in oxr_session.c,
 * which calls xrt_device_update_inputs() (this function) and pushes
 * XrEventDataUserPresenceChangedEXT whenever the returned boolean flips. Only installed
 * as @ref xrt_device::update_inputs when WMR_USER_PRESENCE=1 (see wmr_hmd_create());
 * otherwise the base u_device_noop_update_inputs stays in place, so this is a strict
 * opt-in with no cost or behavior change by default.
 */
static xrt_result_t
wmr_hmd_update_inputs(struct xrt_device *xdev)
{
	struct wmr_hmd *wh = wmr_hmd(xdev);

	for (size_t i = 0; i < wh->base.input_count; i++) {
		struct xrt_input *input = &wh->base.inputs[i];
		if (input->name != XRT_INPUT_GENERIC_HEAD_DETECT) {
			continue;
		}

		// PROVISIONAL threshold (2026-08-18): treat any nonzero raw proximity byte as
		// "worn". This matches the informal convention already used offline by
		// scripts/hmd-watch.py ("PROXIMITY %d -> %d ... WORN if new else removed"),
		// but per docs/22-cable-connector-diagnosis.md that convention itself was
		// never confirmed against a clean live cover/uncover gesture -- treat this as
		// a best guess, not a validated calibration, until a real donning/doffing
		// session confirms whether the sensor is genuinely binary or an analog value
		// that merely idles at 0. The raw value is always logged on change (below) so
		// a real threshold can be picked from live data without a rebuild.
		bool raw_worn = wh->proximity_sensor != 0;
		uint64_t now_ns = os_monotonic_get_ns();

		/*
		 * Debounce (T224). The raw byte alternated 0,1,0,1 through a measured donning
		 * gesture, so a candidate state has to hold for its window before it is committed.
		 * Windows are asymmetric: see the struct comment in wmr_hmd.h for why leaving
		 * "worn" deliberately costs more evidence than entering it.
		 */
		if (raw_worn != wh->presence.candidate) {
			wh->presence.candidate = raw_worn;
			wh->presence.candidate_since_ns = now_ns;
		}

		if (wh->presence.candidate != wh->presence.committed) {
			// Entering "worn" uses the don window, leaving it uses the doff window.
			uint64_t window_ms = (uint64_t)(wh->presence.candidate
			                                    ? debug_get_num_option_wmr_user_presence_don_ms()
			                                    : debug_get_num_option_wmr_user_presence_doff_ms());
			uint64_t held_ns = now_ns - wh->presence.candidate_since_ns;
			if (held_ns >= window_ms * U_TIME_1MS_IN_NS) {
				wh->presence.committed = wh->presence.candidate;
				WMR_INFO(wh, "User presence: %s (raw proximity sensor value %u, held %llu ms)",
				         wh->presence.committed ? "WORN" : "NOT WORN", wh->proximity_sensor,
				         (unsigned long long)(held_ns / U_TIME_1MS_IN_NS));
			}
		}

		/*
		 * Stale-channel notice. The committed state is deliberately NOT touched here: when
		 * the companion dies the last state stands, which for a worn headset is the safe
		 * direction (doff-to-pause quietly stops working rather than pausing a live
		 * session). But a consumer deciding anything on presence deserves to know the
		 * channel went quiet, because T224 measured the companion surviving only ~1-3
		 * minutes per session on degraded hardware -- long enough to look healthy at launch
		 * and be gone by the time anyone tests the feature.
		 */
		const uint64_t PRESENCE_STALE_NS = 30 * U_TIME_1S_IN_NS;
		if (wh->presence.last_update_ns != 0 && (now_ns - wh->presence.last_update_ns) > PRESENCE_STALE_NS) {
			wh->presence.stale_log_count++;
			if (wh->presence.stale_log_count == 1 || (wh->presence.stale_log_count % 5000) == 0) {
				WMR_INFO(wh,
				         "User presence: no proximity update for %llu s -- holding '%s'. The "
				         "companion channel is the dependency here; if it is storming, this "
				         "feature is effectively frozen, not reporting.",
				         (unsigned long long)((now_ns - wh->presence.last_update_ns) / U_TIME_1S_IN_NS),
				         wh->presence.committed ? "WORN" : "NOT WORN");
			}
		}

		input->value.boolean = wh->presence.committed;
		input->timestamp = now_ns;
		break;
	}

	return XRT_SUCCESS;
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
               struct xrt_device **out_right_controller)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	int ret = 0;
	int i;
	int eye;

	// See the DEBUG_GET_ONCE_BOOL_OPTION comment above: default off, zero regression risk.
	bool user_presence_enabled = debug_get_bool_option_wmr_user_presence();
	size_t input_count = user_presence_enabled ? 2 : 1;

	struct wmr_hmd *wh = U_DEVICE_ALLOCATE(struct wmr_hmd, flags, input_count, 0);
	if (!wh) {
		return;
	}

	// Populate the base members.
	wh->base.update_inputs = user_presence_enabled ? wmr_hmd_update_inputs : u_device_noop_update_inputs;
	wh->base.get_tracked_pose = wmr_hmd_get_tracked_pose;
	wh->base.get_view_poses = u_device_get_view_poses;
	wh->base.destroy = wmr_hmd_destroy;
	wh->base.get_compositor_info = get_compositor_info_wmr;
	wh->base.name = XRT_DEVICE_GENERIC_HMD;
	wh->base.device_type = XRT_DEVICE_TYPE_HMD;
	wh->log_level = log_level;

	wh->base.supported.compositor_info = true;

	wh->hid_hololens_sensors_dev = hid_holo;
	wh->hid_control_dev = hid_ctrl;

	// Remembered so the read thread can find the companion again after it re-enumerates.
	// Taken from the prober device rather than hardcoded: the companion's VID/PID differ per
	// headset model (see wmr_prober.c's check_and_get_interface), and a G2 constant here would
	// quietly disable recovery on every other WMR headset.
	if (dev_companion != NULL) {
		wh->companion_vid = dev_companion->vendor_id;
		wh->companion_pid = dev_companion->product_id;
	}

	// Mutex before thread.
	ret = os_mutex_init(&wh->fusion.mutex);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to init fusion mutex!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	ret = os_mutex_init(&wh->hid_lock);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to init HID mutex!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	ret = os_mutex_init(&wh->controller_status_lock);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to init Controller status mutex!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	ret = os_cond_init(&wh->controller_status_cond);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to init Controller status cond!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	// Thread and other state.
	ret = os_thread_helper_init(&wh->oth);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to init threading!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	// Setup input.
	wh->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	if (user_presence_enabled) {
		wh->base.inputs[1].name = XRT_INPUT_GENERIC_HEAD_DETECT;
		wh->base.supported.presence = true;
		WMR_INFO(wh,
		         "User presence enabled (WMR_USER_PRESENCE=1): surfacing the companion "
		         "proximity sensor as XR_EXT_user_presence. Threshold is provisional, see "
		         "wmr_hmd_update_inputs().");
	}

	// Read config file from HMD
	if (wmr_read_config(wh) < 0) {
		WMR_ERROR(wh, "Failed to load headset configuration!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	wh->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	wh->offset = (struct xrt_pose)XRT_POSE_IDENTITY;
	wh->average_imus = true;
	wh->tracked_offset_ms = (struct u_var_draggable_f32){
	    .val = 0.0,
	    .min = -40.0,
	    .step = 0.1,
	    .max = +120.0,
	};

	/* Now that we have the config loaded, iterate the map of known headsets and see if we have
	 * an entry for this specific headset (otherwise the generic entry will be used)
	 */
	for (i = 0; i < headset_map_n; i++) {
		const struct wmr_headset_descriptor *cur = &headset_map[i];

		if (hmd_type == cur->hmd_type) {
			wh->hmd_desc = cur;
			if (hmd_type != WMR_HEADSET_GENERIC)
				break; /* Stop checking if we have a specific match, or keep going for the GENERIC
				          catch-all type */
		}

		if (cur->dev_id_str && strncmp(wh->config_hdr.name, cur->dev_id_str, 64) == 0) {
			hmd_type = cur->hmd_type;
			wh->hmd_desc = cur;
			break;
		}
	}
	assert(wh->hmd_desc != NULL); /* Each supported device MUST have a manually created entry in our headset_map */

	WMR_INFO(wh, "Found WMR headset type: %s", wh->hmd_desc->debug_name);

	wmr_config_precompute_transforms(&wh->config.sensors, wh->config.eye_params);

	struct u_extents_2d exts;
	exts.w_pixels = (uint32_t)wh->config.eye_params[0].display_size.x;
	exts.h_pixels = (uint32_t)wh->config.eye_params[0].display_size.y;
	u_extents_2d_split_side_by_side(&wh->base, &exts);

	// WMR panels (Reverb G1/G2, Odyssey, Odyssey+, ...) run at 90 Hz.
	// u_extents_2d_split_side_by_side() does not set the frame interval, so
	// without this it stays 0 — and a downstream consumer (the SteamVR driver
	// bridge) computes refresh = 1/0 = infinite and falls back to 60 Hz,
	// which mis-paces frames against the 90 Hz panel and causes judder.
	wh->base.hmd->screens[0].nominal_frame_interval_ns =
	    (uint64_t)(1000000000.0 / 90.0);

	// Fill in blend mode - just opqaue, unless we get Hololens support one day.
	size_t idx = 0;
	wh->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	wh->base.hmd->blend_mode_count = idx;

	wh->config.eye_params[0].poly_3k.y_offset = debug_get_num_option_left_view_y_offset();
	wh->config.eye_params[1].poly_3k.y_offset = debug_get_num_option_right_view_y_offset();

	// Distortion information, fills in xdev->compute_distortion().
	for (eye = 0; eye < 2; eye++) {
		struct xrt_fov *fov = &wh->base.hmd->distortion.fov[eye];
		struct u_poly_3k_eye_values *poly_3k = &wh->config.eye_params[eye].poly_3k;

		u_compute_distortion_bounds_poly_3k(&poly_3k->inv_affine_xform, poly_3k->channels, eye, fov,
		                                    &poly_3k->tex_x_range, &poly_3k->tex_y_range);

		WMR_INFO(wh, "FoV eye %d angles left %f right %f down %f up %f", eye, fov->angle_left, fov->angle_right,
		         fov->angle_down, fov->angle_up);

		WMR_INFO(wh, "Render texture range %f, %f to %f, %f", poly_3k->tex_x_range.x, poly_3k->tex_y_range.x,
		         poly_3k->tex_x_range.y, poly_3k->tex_y_range.y);
	}

	wh->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	wh->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	wh->base.compute_distortion = compute_distortion_wmr;
	u_distortion_mesh_fill_in_compute(&wh->base);

	// Set initial HMD screen power state.
	wh->hmd_screen_enable = true;

	/* We're set up. Activate the HMD and turn on the IMU */
	if (wh->hmd_desc->init_func && wh->hmd_desc->init_func(wh) != 0) {
		WMR_ERROR(wh, "Activation of HMD failed");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	// Switch on IMU on the HMD.
	hololens_sensors_enable_imu(wh);

	if (debug_get_bool_option_wmr_cameras() && debug_get_bool_option_wmr_constellation_controllers()) {
		wmr_hmd_create_constellation_tracker(wh);
	}

	// Switch on data streams on the HMD (only cameras for now as IMU is not yet integrated into wmr_source)
	wh->tracking.source = wmr_source_create(&wh->tracking.xfctx, dev_holo, wh->config,
	                                        wh->tracking.constellation_tracker != NULL
	                                            ? wh->tracking.constellation_cam_blob_sinks
	                                            : NULL);

	struct xrt_slam_sinks sinks = {0};
	struct xrt_device *hand_device = NULL;
	bool success = wmr_hmd_setup_trackers(wh, &sinks, &hand_device);
	if (!success) {
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	// Stream data source into sinks (if populated).
	//
	// With WMR_CAMERAS=0 the stream is not started at all: setup_trackers has already
	// forced SLAM and hand tracking off, so nothing consumes the frames, and skipping the
	// start also means a camera in a bad state cannot abort an orientation-only session
	// (a failed stream start below destroys the whole HMD). The camera USB interface is
	// still claimed by wmr_source_create above; only streaming is skipped.
	if (!debug_get_bool_option_wmr_cameras()) {
		WMR_INFO(wh, "Camera streaming disabled (WMR_CAMERAS=0), running orientation-only");
	} else {
		bool stream_started = xrt_fs_slam_stream_start(wh->tracking.source, &sinks);
		if (!stream_started) {
			//! @todo Could reach this due to !XRT_HAVE_LIBUSB but the HMD should keep working
			WMR_WARN(wh, "Failed to start WMR source");
			wmr_hmd_destroy(&wh->base);
			wh = NULL;
			return;
		}
	}

	// Hand over hololens sensor device to reading thread.
	ret = os_thread_helper_start(&wh->oth, wmr_run_thread, wh);
	if (ret != 0) {
		WMR_ERROR(wh, "Failed to start thread!");
		wmr_hmd_destroy(&wh->base);
		wh = NULL;
		return;
	}

	/* Send controller status request to check for online controllers
	 * and wait 250ms for the reports for Reverb G2 and Odyssey+ */
	if (wh->hmd_desc->hmd_type == WMR_HEADSET_REVERB_G2 || wh->hmd_desc->hmd_type == WMR_HEADSET_SAMSUNG_800ZAA) {
		bool have_controller_status = false;

		os_mutex_lock(&wh->controller_status_lock);
		if (wmr_hmd_request_controller_status(wh)) {
			/* The reader thread sets one flag per processed status report (which
			 * includes creating that controller when it is online) and signals the
			 * cond once both are set. Wait for BOTH - exiting on the first would
			 * routinely lose the second controller.
			 *
			 * Bounded: a lost status reply used to hang startup forever here
			 * (resolving the @todo that was in this spot). Polling instead of a
			 * cond wait because os_threading has no timed cond wait yet; while
			 * waiting, re-request the status every second so one lost
			 * request/reply doesn't cost the whole window. The deadline covers
			 * two worst-case controller creations incl. fw-read retries. */
			const int64_t deadline_ns = os_monotonic_get_ns() + (int64_t)10 * U_TIME_1S_IN_NS;
			int64_t next_request_ns = os_monotonic_get_ns() + U_TIME_1S_IN_NS;
			while (!(wh->have_left_controller_status && wh->have_right_controller_status) &&
			       os_monotonic_get_ns() < deadline_ns) {
				os_mutex_unlock(&wh->controller_status_lock);
				os_nanosleep(U_TIME_1MS_IN_NS * 20);
				os_mutex_lock(&wh->controller_status_lock);

				int64_t now_ns = os_monotonic_get_ns();
				if (now_ns >= next_request_ns) {
					wmr_hmd_request_controller_status(wh);
					next_request_ns = now_ns + U_TIME_1S_IN_NS;
				}
			}
			have_controller_status = wh->have_left_controller_status && wh->have_right_controller_status;
		}
		os_mutex_unlock(&wh->controller_status_lock);

		if (!have_controller_status) {
			WMR_WARN(wh, "Failed to request controller status from HMD");
		}
	}

	wmr_hmd_setup_ui(wh);

	*out_hmd = &wh->base;
	*out_handtracker = hand_device;

	os_mutex_lock(&wh->controller_status_lock);
	if (wh->controller[0] != NULL) {
		*out_left_controller = wmr_hmd_controller_connection_get_controller(wh->controller[0]);
	} else {
		*out_left_controller = NULL;
	}

	if (wh->controller[1] != NULL) {
		*out_right_controller = wmr_hmd_controller_connection_get_controller(wh->controller[1]);
	} else {
		*out_right_controller = NULL;
	}
	os_mutex_unlock(&wh->controller_status_lock);
}

bool
wmr_hmd_send_controller_packet(struct wmr_hmd *hmd, const uint8_t *buffer, uint32_t buf_size)
{
	os_mutex_lock(&hmd->hid_lock);
	int ret = os_hid_write(hmd->hid_hololens_sensors_dev, buffer, buf_size);
	os_mutex_unlock(&hmd->hid_lock);

	return ret != -1 && (uint32_t)(ret) == buf_size;
}

/* Called from WMR controller implementation only during fw reads. @todo: Refactor
 * controller firmware reads to happen from a state machine and not require this blocking method */
int
wmr_hmd_read_sync_from_controller(struct wmr_hmd *hmd, uint8_t *buffer, uint32_t buf_size, int timeout_ms)
{
	os_mutex_lock(&hmd->hid_lock);
	int res = os_hid_read(hmd->hid_hololens_sensors_dev, buffer, buf_size, timeout_ms);
	os_mutex_unlock(&hmd->hid_lock);

	return res;
}
