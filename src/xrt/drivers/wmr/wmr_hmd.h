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
	 */
	struct
	{
		//! State currently reported to the app.
		bool committed;
		//! Candidate state waiting out its debounce window, and when it first appeared.
		bool candidate;
		uint64_t candidate_since_ns;
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
	} presence;

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
