// Copyright 2021, Jan Schmidt
// Copyright 2021, Philipp Zabel
// Copyright 2021, Jakob Bornecrantz
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  WMR camera interface
 * @author Jan Schmidt <jan@centricular.com>
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_wmr
 */

#include "math/m_api.h"

#include "os/os_threading.h"
#include "os/os_time.h"
#include "xrt/xrt_byte_order.h"

#include "util/u_autoexpgain.h"
#include "util/u_debug.h"
#include "util/u_linux.h"
#include "util/u_var.h"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_trace_marker.h"

#include "wmr_config.h"
#include "wmr_protocol.h"
#include "wmr_camera.h"

#include <libusb.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <time.h>

//! Specifies whether the user wants to enable autoexposure from the start.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_autoexposure, "WMR_AUTOEXPOSURE", true)

/*
 * reverb-g2 (2026-09-05): dump one throttled raw-grayscale PGM snapshot per SLAM-tracking camera
 * to ~/vr/cameraN.pgm, for the web dashboard's live tracking-camera view (docs precedent:
 * wmr_hmd.c's HMD-temperature dashboard snapshot, same "cheap raw dump here, real encoding in
 * Python at serve time" split -- see status-dashboard.py). Default ON: this thread
 * ("WMR: USB-Camera", see wmr_cam_usb_thread) is NOT elevated to SCHED_FIFO (only "WMR: USB-HMD"
 * is -- grep u_linux_try_to_set_realtime_priority_on_thread), so unlike that thread it has never
 * carried the same real-time obligations; the throttle below (WMR_CAMERA_SNAPSHOT_THROTTLE,
 * ~1 fps) and the fact that the real tracking sinks are always pushed BEFORE this snapshot runs
 * keep it from ever being able to delay a frame's arrival at SLAM/Basalt either way.
 */
DEBUG_GET_ONCE_BOOL_OPTION(wmr_camera_snapshot, "WMR_CAMERA_SNAPSHOT", true)

//! Every Nth SLAM-tracking-frame callback does the real (tiny) dashboard snapshot file I/O; SLAM
//! frames land here at roughly 30fps (see the "Tracking frames usually come at ~30fps" comment
//! below), so 30 throttles the per-camera dump to roughly 1 fps -- nowhere near the ~30-90fps
//! source rate, per the driver's documented sensitivity to added work in this pipeline (docs/44
//! T199: an in-loop sleep() once let the IMU stream fall ~630ms behind). Runtime-overridable
//! (docs/08 v0 passthrough, 2026-09-05): the dashboard thumbnail only ever needed ~1fps, but a
//! live passthrough viewer wants much closer to the source rate. Default stays 30 (unchanged
//! dashboard behaviour) unless WMR_CAMERA_SNAPSHOT_RATE_DIVISOR overrides it; the write itself is
//! already ordered after the real tracking sinks are pushed (see the call site below) so it can
//! never delay a pose reaching SLAM/Basalt -- lowering the divisor only spends more of this
//! thread's own budget on file I/O, which is the thing to watch when raising it.
#define WMR_CAMERA_SNAPSHOT_THROTTLE 30

DEBUG_GET_ONCE_NUM_OPTION(wmr_camera_snapshot_rate_divisor, "WMR_CAMERA_SNAPSHOT_RATE_DIVISOR", WMR_CAMERA_SNAPSHOT_THROTTLE)

//! Specifies whether the user wants to use the same exp/gain values for all cameras
DEBUG_GET_ONCE_BOOL_OPTION(wmr_unify_expgain, "WMR_UNIFY_EXPGAIN", false)

//! Mirrors the option of the same name in wmr_hmd.c -- this is unvalidated, opt-in only.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_constellation_controllers, "WMR_CONSTELLATION_CONTROLLERS", false)

/*
 * reverb-g2 patch 0101 (2026-08-27): stamp SLAM frames at mid-EXPOSURE instead of mid-SLOT.
 * The frame footer's end_ts - start_ts is the 90 Hz slot period (~11.1 ms), not the exposure,
 * so `start + delta/2` puts the stamp 5.55 ms after the exposure start while the real exposure
 * (pixel header, microseconds, auto 60-9000) is centred at start + exposure/2 -- i.e. the
 * frame reaches Basalt 1-5.5 ms late relative to the IMU, and an offline sweep of the wearer's
 * yaw recording (docs/80) showed that lateness is the single biggest position-drift lever on
 * fast head turns (-7 ms fixed shift: drift 0.96 -> 0.24 m on config I). This follows the
 * exposure frame by frame instead of a fixed shift. SLAM frames only; controller frames keep
 * the mid-slot stamp (the constellation tracker has no measurement to calibrate against yet).
 * Default off: current behaviour unchanged unless WMR_CAM_TS_MID_EXPOSURE=1.
 */
DEBUG_GET_ONCE_BOOL_OPTION(wmr_cam_ts_mid_exposure, "WMR_CAM_TS_MID_EXPOSURE", false)

static int
update_expgain(struct wmr_camera *cam, struct xrt_frame **frames);
static int
wmr_camera_set_ctrl_exposure_gain(struct wmr_camera *cam, uint8_t camera_id, uint16_t exposure, uint8_t gain);

/*
 *
 * Defines and structs.
 *
 */

#define WMR_CAM_TRACE(c, ...) U_LOG_IFL_T((c)->log_level, __VA_ARGS__)
#define WMR_CAM_DEBUG(c, ...) U_LOG_IFL_D((c)->log_level, __VA_ARGS__)
#define WMR_CAM_INFO(c, ...) U_LOG_IFL_I((c)->log_level, __VA_ARGS__)
#define WMR_CAM_WARN(c, ...) U_LOG_IFL_W((c)->log_level, __VA_ARGS__)
#define WMR_CAM_ERROR(c, ...) U_LOG_IFL_E((c)->log_level, __VA_ARGS__)

#define CAM_ENDPOINT 0x05

#define NUM_XFERS 9

#define WMR_CAMERA_CMD_GAIN 0x80
#define WMR_CAMERA_CMD_ON 0x81
#define WMR_CAMERA_CMD_OFF 0x82

#define DEFAULT_EXPOSURE 6000
/*
 * Controller-tracking frames want a very short exposure and minimum gain: the point is to see
 * the controllers' own (visible-light) LEDs as isolated bright points and essentially nothing
 * else. Values from thaytan's dev-constellation-controller-tracking branch, which is the
 * reference implementation for WMR constellation tracking.
 *
 * This is also why WMR controller tracking works in a dark room and why using these headsets
 * in direct sunlight is advised against: the exposure is chosen so that only the LEDs stand
 * out, so ambient light is irrelevant until it is bright enough to swamp them.
 */
// EXPERIMENT 2026-08-11: thaytan's values (400/1) read back as exposure=20 on this hardware once
// the +2 slot fix landed -- still a fully black debug panel. Trying much higher values to check
// whether this is simply an amplitude problem before doubting the slot mapping itself.
#define DEFAULT_CTRL_EXPOSURE 6000
#define DEFAULT_CTRL_GAIN 100
#define DEFAULT_GAIN 127

/*
 * Manual override for the controller-tracking frames' FIXED exposure/gain (they never
 * adapt -- WMR_AUTOEXPOSURE only drives the SLAM frames). Motivation (T221, 2026-08-19):
 * in-motion constellation acceptance collapsed to ~zero while at-rest acceptance ran at
 * thousands of samples -- 6 ms of exposure on a fast-moving hand smears the LED points
 * into arcs, corrupting centroids and correspondence, and fresh cells may bloom them.
 * A/B knob for the motion-blur/bloom hypothesis; defaults preserve current behavior
 * exactly. Hardware range observed on the G2 (see wmr_camera_gain_cmd): exposure 60-9000,
 * gain 16-255. Untested values below ~60 read back clamped (2026-08-11 experiment above).
 */
DEBUG_GET_ONCE_NUM_OPTION(wmr_ctrl_exposure, "WMR_CONTROLLER_CAM_EXPOSURE_US", DEFAULT_CTRL_EXPOSURE)
DEBUG_GET_ONCE_NUM_OPTION(wmr_ctrl_gain, "WMR_CONTROLLER_CAM_GAIN", DEFAULT_CTRL_GAIN)

//! Passthrough-viewer support (reverb-g2, 2026-09-05): the controller-tracking exposure/gain
//! above is a fixed pair chosen for LED visibility, completely independent of the SLAM
//! autoexposure loop in update_expgain() -- normal and harmless when the two streams are only
//! ever consumed separately (real SLAM vs real controller tracking), but once
//! WMR_CAMERA_SNAPSHOT_RATE_DIVISOR=1 makes both frame types land in the SAME dashboard/
//! passthrough PGM file (docs/08 v0), alternating between two independently-exposed sources
//! shows up as a visible brightness flicker. Default OFF: this must never silently change
//! exposure behaviour for existing controller-tracking use (Aircar/Cyberpilot booth titles).
//! When on, ties the controller slots' exposure/gain to camera 0's current SLAM autoexposure
//! value instead of the fixed default -- see the call site in update_expgain().
DEBUG_GET_ONCE_BOOL_OPTION(wmr_ctrl_exposure_follow_slam, "WMR_CTRL_EXPOSURE_FOLLOW_SLAM", false)

#define WMR_FRAMETYPE_SLAM 0x0
#define WMR_FRAMETYPE_CONTROLLER 0x2

#define WMR_DEBUG_SINK_SLAM 0
#define WMR_DEBUG_SINK_CONTROLLER 1

struct wmr_camera_active_cmd
{
	__le32 magic;
	__le32 len;
	__le32 cmd;
} __attribute__((packed));

struct wmr_camera_gain_cmd
{
	__le32 magic;
	__le32 len;
	__le16 cmd;
	__le16 camera_id;
	__le16 exposure;   //!< observed 60 to 6000 (but supports up to ~9000)
	__le16 gain;       //!< observed 16 to 255
	__le16 camera_id2; //!< same as camera_id
} __attribute__((packed));

struct wmr_camera
{
	libusb_context *ctx;
	libusb_device_handle *dev;

	bool running;

	struct os_thread_helper usb_thread;
	int usb_complete;

	struct wmr_camera_config tcam_confs[WMR_MAX_CAMERAS]; //!< Configs for tracking cameras
	int tcam_count;                                       //!< Number of tracking cameras
	int slam_cam_count;                                   //!< Number of tracking cameras used for SLAM

	size_t xfer_size;
	size_t frame_xfer_size;
	uint32_t frame_width, frame_height;
	uint8_t last_seq;
	uint64_t last_frame_ts;

	/* Unwrapped frame sequence number */
	uint64_t frame_sequence;

	struct libusb_transfer *xfers[NUM_XFERS];

	struct wmr_camera_expgain
	{
		bool manual_control; //!< Whether to control exp/gain manually or with aeg
		uint16_t last_exposure, exposure;
		uint8_t last_gain, gain;
		struct u_var_draggable_u16 exposure_ui; //! Widget to control `exposure` value
		struct u_autoexpgain *aeg;
	} ceg[WMR_MAX_CAMERAS]; //!< Camera exposure-gain control
	bool unify_expgains;    //!< Whether to use the same exposure/gain values for all cameras

	struct u_sink_debug debug_sinks[2];

	struct xrt_frame_sink *cam_sinks[WMR_MAX_CAMERAS]; //!< Downstream sinks to push tracking frames to
	struct xrt_frame_sink *ctrl_cam_sinks[WMR_MAX_CAMERAS]; //!< Downstream sinks for controller-tracking frames

	bool snapshot_enabled;          //!< Dashboard PGM snapshot feature on/off (WMR_CAMERA_SNAPSHOT)
	uint64_t snapshot_frame_count;  //!< Throttle counter for that snapshot (reverb-g2, 2026-09-05)
	//! Separate throttle counter for the controller-tracking-frame snapshot below (reverb-g2,
	//! 2026-09-05): the two frame types arrive interleaved on the same USB stream (SLAM
	//! ~30fps, controller ~60fps, ~90fps raw combined -- see the frametype comment in
	//! img_xfer_cb()), so each needs its own independent modulo counter rather than sharing one.
	uint64_t ctrl_snapshot_frame_count;

	/*!
	 * Real frame drop-rate (reverb-g2, 2026-09-05). Under normal operation each mosaic frame
	 * callback advances the hardware's 8-bit `seq` counter by exactly 1; a bigger jump means
	 * (seq_delta - 1) earlier tick(s) never arrived as a frame at all (USB stall/backpressure,
	 * not just a slow callback). See the seq/seq_delta computation in img_xfer_cb().
	 */
	uint32_t dropped_frame_count;
	//! Guards @ref dropped_frame_count against a false hit on the very first frame this struct
	//! ever processes, whose `last_seq` is still the zeroed default rather than a real previous
	//! tick (so its seq_delta is meaningless, not a real gap).
	bool have_prev_seq;

	//! Throttle counter for the exposure/gain dashboard snapshot, incremented once per
	//! update_expgain() call (reverb-g2, 2026-09-05) -- that's once per SLAM frame, ~30 Hz, same
	//! cadence the WMR_CAMERA_SNAPSHOT feature throttles against.
	uint64_t expgain_snapshot_count;

	//! Fixed controller-tracking exposure/gain (WMR_CONSTELLATION_CONTROLLERS), set once at
	//! open() time -- see wmr_camera_set_ctrl_exposure_gain(). Same pair for every camera, so
	//! unlike `ceg[]` this isn't per-camera. Only valid when ctrl_expgain_set is true.
	uint16_t ctrl_exposure;
	uint8_t ctrl_gain;
	bool ctrl_expgain_set;

	enum u_logging_level log_level;
};


/*
 *
 * Helper functions.
 *
 */

/* Some WMR headsets use 616538 byte transfers. HP G2 needs 1233018 (4 cameras)
 * As a general formula, it seems we have:
 *   0x6000 byte packets. Each has a 32 byte header.
 *     packet contains frame data for each camera in turn.
 *     Each frame has an extra (first) line with metadata
 *   Then, there's an extra 26 bytes on the end.
 *
 *   F = camera frames X * (Y+1) + 26
 *   n_packets = F/(0x6000-32)
 *   leftover = F - n_packets*(0x6000-32)
 *   size = n_packets * 0x6000 + 32 + leftover,
 *
 *   so for 2 x 640x480 cameras:
 *			F = 2 * 640 * 481 + 26 = 615706
 *      n_packets = 615706 / 24544 = 25
 *      leftover = 615706 - 25 * 24544 = 2106
 *      size = 25 * 0x6000 + 32 + 2106 = 616538
 *
 *  For HP G2 = 4 x 640 * 480 cameras:
 *			F = 4 * 640 * 481 + 26 = 1231386
 *      n_packets = 1231386 / 24544 = 50
 *      leftover = 1231386 - 50 * 24544 = 4186
 *      size = 50 * 0x6000 + 32 + 4186 = 1233018
 *
 *  It would be good to test these calculations on other headsets with
 *  different camera setups.
 */
static bool
compute_frame_size(struct wmr_camera *cam)
{
	int i;
	int cams_found = 0;
	int width;
	int height;
	size_t F;
	size_t n_packets;
	size_t leftover;

	F = 26;

	libusb_device *dev = libusb_get_device(cam->dev);
	int ep_pkt_size = libusb_get_max_packet_size(dev, CAM_ENDPOINT);
	if (ep_pkt_size < 0) {
		WMR_CAM_ERROR(cam, "Failed to retrieve endpoint descriptor: result %d", ep_pkt_size);
		return false;
	}

	for (i = 0; i < cam->tcam_count; i++) {
		const struct wmr_camera_config *config = &cam->tcam_confs[i];

		WMR_CAM_DEBUG(cam, "Found head tracking camera index %d width %d height %d", i, config->roi.extent.w,
		              config->roi.extent.h);

		if (cams_found == 0) {
			width = config->roi.extent.w;
			height = config->roi.extent.h;
		} else if (height != config->roi.extent.h) {
			WMR_CAM_ERROR(cam, "Head tracking sensors have mismatched heights - %u != %u. Please report",
			              height, config->roi.extent.h);
			return false;
		} else {
			width += config->roi.extent.w;
		}

		cams_found++;
		F += config->roi.extent.w * (config->roi.extent.h + 1);
	}

	if (cams_found == 0) {
		return false;
	}

	if (width < 1280 || height < 480) {
		return false;
	}

	n_packets = F / (0x6000 - 32);
	leftover = F - n_packets * (0x6000 - 32);

	cam->frame_xfer_size = n_packets * 0x6000 + 32 + leftover;

	// Round up to a multiple of the max packet size to avoid overflows
	// in case of stalls reading things out
	size_t round_up = ep_pkt_size - (cam->frame_xfer_size % ep_pkt_size);
	cam->xfer_size = cam->frame_xfer_size + round_up;
	WMR_CAM_DEBUG(cam, "Rounding up xfer size by %zu from frame size %zu to %zu", round_up, cam->frame_xfer_size,
	              cam->xfer_size);

	cam->frame_width = width;
	cam->frame_height = height;

	WMR_CAM_INFO(cam, "WMR camera framebuffer %u x %u - %zu transfer size", cam->frame_width, cam->frame_height,
	             cam->xfer_size);

	return true;
}

static void *
wmr_cam_usb_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("WMR: USB-Camera");

	struct wmr_camera *cam = ptr;

#ifdef XRT_OS_LINUX
	// docs/08 (2026-09-05): under Basalt's CPU/scheduling pressure this thread misses its
	// SCHED_OTHER slot often enough that real USB camera transfers get dropped before the
	// driver's frame callback ever runs. Mirrors "WMR: USB-HMD"'s existing SCHED_FIFO
	// elevation (wmr_hmd.c); opt out with WMR_CAMERA_THREAD_NO_RT=1.
	if (getenv("WMR_CAMERA_THREAD_NO_RT") == NULL) {
		u_linux_try_to_set_realtime_priority_on_thread(cam->log_level, "WMR: USB-Camera");
	}
#endif

	os_thread_helper_lock(&cam->usb_thread);
	while (os_thread_helper_is_running_locked(&cam->usb_thread) && !cam->usb_complete) {
		os_thread_helper_unlock(&cam->usb_thread);

		libusb_handle_events_completed(cam->ctx, &cam->usb_complete);

		os_thread_helper_lock(&cam->usb_thread);
	}

	//! @todo Think this is not needed? what condition are we waiting for?
	os_thread_helper_wait_locked(&cam->usb_thread);
	os_thread_helper_unlock(&cam->usb_thread);

	return NULL;
}

static int
send_buffer_to_device(struct wmr_camera *cam, uint8_t *buf, uint8_t len)
{
	struct libusb_transfer *xfer;
	uint8_t *data;

	xfer = libusb_alloc_transfer(0);
	if (xfer == NULL) {
		return LIBUSB_ERROR_NO_MEM;
	}

	data = malloc(len);
	if (data == NULL) {
		libusb_free_transfer(xfer);
		return LIBUSB_ERROR_NO_MEM;
	}

	memcpy(data, buf, len);

	libusb_fill_bulk_transfer(xfer, cam->dev, CAM_ENDPOINT | LIBUSB_ENDPOINT_OUT, data, len, NULL, NULL, 0);
	xfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;

	return libusb_submit_transfer(xfer);
}

static int
set_active(struct wmr_camera *cam, bool active)
{
	struct wmr_camera_active_cmd cmd = {
	    .magic = __cpu_to_le32(WMR_MAGIC),
	    .len = __cpu_to_le32(sizeof(struct wmr_camera_active_cmd)),
	    .cmd = __cpu_to_le32(active ? WMR_CAMERA_CMD_ON : WMR_CAMERA_CMD_OFF),
	};

	return send_buffer_to_device(cam, (uint8_t *)&cmd, sizeof(cmd));
}

/*!
 * Dump one camera's already-demuxed frame as a raw 8-bit grayscale PGM (P5) to
 * ~/vr/camera<index>.pgm, atomically (write to a .tmp file, then rename()), for the web
 * dashboard's live tracking-camera view. Best-effort only: any failure just warns once (a static
 * guard, matching wmr_hmd.c's HMD-temperature snapshot) and returns -- this must never be allowed
 * to affect the driver's real job.
 *
 * `xf` is one of the per-camera ROI frames already produced by u_frame_create_roi() in
 * img_xfer_cb() below: `xf->width`/`xf->height` are this camera's true dimensions (640x480 on the
 * G2), but `xf->stride` is still the COMBINED multi-camera frame's stride (all tcam_count cameras
 * side by side), not this camera's own width -- so rows are copied one at a time, respecting
 * stride, rather than as one contiguous `width * height`-byte block (which would interleave in
 * neighbouring cameras' columns into every row after the first).
 */
static void
wmr_camera_dump_snapshot_pgm(struct wmr_camera *cam, int index, struct xrt_frame *xf)
{
	static bool warned_snapshot_failure = false;

	// Latency instrumentation (reverb-g2, 2026-09-05, docs/08 v0 passthrough redraw
	// investigation): stamped here, at the moment this driver thread hands the frame to
	// disk, using the same os_monotonic_get_ns() (CLOCK_MONOTONIC) clock the rest of Monado
	// runs on -- safe to diff against a reader process's own CLOCK_MONOTONIC read on the same
	// host, unlike the device's own HoloLens-tick timestamps (frame_start_ts et al above),
	// which are a separate, uncalibrated clock domain. Written as a sidecar, not embedded in
	// the PGM itself, so existing readers (the dashboard) are unaffected.
	const int64_t snapshot_write_ts_ns = os_monotonic_get_ns();

	const char *home = getenv("HOME");
	if (home == NULL) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Camera snapshot: HOME not set, skipping dashboard snapshot file");
			warned_snapshot_failure = true;
		}
		return;
	}

	char final_path[PATH_MAX];
	char tmp_path[PATH_MAX];
	snprintf(final_path, sizeof(final_path), "%s/vr/camera%d.pgm", home, index);
	snprintf(tmp_path, sizeof(tmp_path), "%s/vr/camera%d.pgm.tmp", home, index);

	FILE *snap = fopen(tmp_path, "wb");
	if (snap == NULL) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Camera snapshot: fopen(%s) failed: %s", tmp_path, strerror(errno));
			warned_snapshot_failure = true;
		}
		return;
	}

	fprintf(snap, "P5\n%u %u\n255\n", xf->width, xf->height);

	bool write_ok = true;
	const uint8_t *row = xf->data;
	for (uint32_t y = 0; y < xf->height; y++) {
		if (fwrite(row, 1, xf->width, snap) != xf->width) {
			write_ok = false;
			break;
		}
		row += xf->stride;
	}

	if (fclose(snap) != 0) {
		write_ok = false;
	}

	if (!write_ok) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Camera snapshot: failed writing %s: %s", tmp_path, strerror(errno));
			warned_snapshot_failure = true;
		}
		unlink(tmp_path);
		return;
	}

	if (rename(tmp_path, final_path) != 0) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Camera snapshot: rename(%s -> %s) failed: %s", tmp_path, final_path,
			             strerror(errno));
			warned_snapshot_failure = true;
		}
		return;
	}

	// Best-effort sidecar with the write timestamp (see the comment at function entry).
	// Deliberately silent on failure -- this is instrumentation only, must never warn louder
	// than the real dump it rides alongside.
	char ts_final_path[PATH_MAX];
	char ts_tmp_path[PATH_MAX];
	snprintf(ts_final_path, sizeof(ts_final_path), "%s/vr/camera%d.pgm.ts", home, index);
	snprintf(ts_tmp_path, sizeof(ts_tmp_path), "%s/vr/camera%d.pgm.ts.tmp", home, index);
	FILE *ts_file = fopen(ts_tmp_path, "wb");
	if (ts_file != NULL) {
		fprintf(ts_file, "%lld\n", (long long)snapshot_write_ts_ns);
		if (fclose(ts_file) == 0) {
			rename(ts_tmp_path, ts_final_path);
		} else {
			unlink(ts_tmp_path);
		}
	}
}

static void LIBUSB_CALL
img_xfer_cb(struct libusb_transfer *xfer)
{
	DRV_TRACE_MARKER();

	struct wmr_camera *cam = xfer->user_data;

	if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
		WMR_CAM_DEBUG(cam, "Camera transfer completed with status: %s (%u)", libusb_error_name(xfer->status),
		              xfer->status);
		goto out;
	}

	if ((size_t)xfer->actual_length < cam->frame_xfer_size) {
		WMR_CAM_DEBUG(cam, "Camera transfer only delivered %d bytes of %zu per frame", xfer->actual_length,
		              cam->frame_xfer_size);
		goto out;
	}

	WMR_CAM_TRACE(cam, "Camera transfer complete - %d bytes of %d", xfer->actual_length, xfer->length);

	/* Convert the output into frames and send them off to debug / tracking */
	struct xrt_frame *xf = NULL;

	/* There's always one extra line of pixels with exposure info */
	u_frame_create_one_off(XRT_FORMAT_L8, cam->frame_width, cam->frame_height + 1, &xf);

	const uint8_t *src = xfer->buffer;

	uint8_t *dst = xf->data;
	size_t dst_remain = xf->size;
	const size_t chunk_size = 0x6000 - 32;

	DRV_TRACE_BEGIN(copy_to_frame);
	while (dst_remain >= 0x20) {
		const size_t to_copy = dst_remain > chunk_size ? chunk_size : dst_remain;

		/* 32 byte header seems to contain:
		 *   __be32 magic = "Dlo+"
		 *   __le32 frame_ctr;
		 *   __le32 slice_ctr;
		 *   __u8 unknown[20]; - binary block where all bytes are different each slice,
		 *                       but repeat every 8 slices. They're different each boot
		 *                       of the headset. Might just be uninitialised memory?
		 */
		uint32_t magic = __le32_to_cpu(*(__le32 *)(src));
		if (magic != WMR_MAGIC) {
			WMR_CAM_WARN(cam, "Invalid frame magic (got %x, expected %x). Dropping", magic, WMR_MAGIC);
			goto drop_frame;
		}
		src += 0x20;

		memcpy(dst, src, to_copy);
		src += to_copy;
		dst += to_copy;
		dst_remain -= to_copy;
	}
	DRV_TRACE_END(copy_to_frame);

	/* There should be exactly a 26 byte footer left over if we completely consumed the right amount of data */
	if (xfer->buffer + cam->frame_xfer_size - src != 26) {
		WMR_CAM_WARN(cam, "Invalid frame. Dropping");
		goto drop_frame;
	}

	/* Footer contains:
	 * __le64 start_ts; - 100ns unit timestamp, from same clock as video_timestamps on the IMU feed
	 * __le64 end_ts;   - 100ns unit timestamp, always about 111000 * 100ns later than start_ts ~= 90Hz
	 * __le16 ctr1;     - Counter that increments by 88, but sometimes by 96, and wraps at 16384
	 * __le16 unknown0  - Unknown value, has only ever been 0
	 * __be32 magic     - "Dlo+"
	 * __le16 frametype?- either 0x00 or 0x02. Every 3rd frame is 0x0, others are 0x2. Might be SLAM vs controllers?
	 */
	uint64_t frame_start_ts = read64(&src) * WMR_MS_HOLOLENS_NS_PER_TICK;
	uint64_t frame_end_ts = read64(&src) * WMR_MS_HOLOLENS_NS_PER_TICK;
	int64_t delta = frame_end_ts - frame_start_ts;

	uint16_t unknown16 = read16(&src);
	uint16_t unknown16_2 = read16(&src);
	src += 4; // Skip "Dlo+" magic bytes
	uint16_t frametype = read16(&src);
	/* frametype 0 is SLAM, frametype 2 is controller tracking */
	bool slam_tracking_frame = (frametype == WMR_FRAMETYPE_SLAM);

	WMR_CAM_TRACE(cam,
	              "Frame start TS %" PRIu64 " (%" PRIi64 " since last) end %" PRIu64 " dt %" PRIi64
	              " unknown %u %u frame type %u",
	              frame_start_ts, frame_start_ts - cam->last_frame_ts, frame_end_ts, delta, unknown16, unknown16_2,
	              frametype);

	/* Read values from the pixel header */
	uint16_t exposure = xf->data[6] << 8 | xf->data[7];
	uint8_t seq = xf->data[89];
	uint8_t seq_delta = seq - cam->last_seq;

	// Real frame drop-rate (reverb-g2, 2026-09-05): this callback should see seq_delta == 1
	// every time -- one hardware tick per mosaic frame delivered. A bigger jump means
	// (seq_delta - 1) tick(s) in between never showed up as a frame here at all. Guarded by
	// have_prev_seq so the very first frame this struct ever processes -- whose last_seq is
	// just the zeroed default, not a real previous tick -- can't manufacture a bogus one-time
	// drop count on startup.
	if (cam->have_prev_seq && seq_delta > 1) {
		cam->dropped_frame_count += (seq_delta - 1);
	}
	cam->have_prev_seq = true;

	/* Extend the sequence number to 64-bits */
	cam->frame_sequence += seq_delta;

	WMR_CAM_TRACE(cam, "Camera frame seq %u (prev %u) -> frame %" PRIu64 " - exposure %u", seq, cam->last_seq,
	              cam->frame_sequence, exposure);

	xf->source_sequence = cam->frame_sequence;
	xf->timestamp = frame_start_ts + delta / 2;
	xf->source_timestamp = frame_start_ts;

	// reverb-g2 0101 (see the option's comment at the top of the file).
	if (slam_tracking_frame && debug_get_bool_option_wmr_cam_ts_mid_exposure()) {
		uint64_t mid_exposure_ts = frame_start_ts + ((uint64_t)exposure * 1000) / 2;
		WMR_CAM_TRACE(cam, "mid-exposure stamp: exposure %u us, start+slot/2 %" PRIi64 " -> start+exposure/2 %" PRIu64 " (%" PRIi64 " ns earlier)",
		              exposure, xf->timestamp, mid_exposure_ts, xf->timestamp - (int64_t)mid_exposure_ts);
		xf->timestamp = (int64_t)mid_exposure_ts;
	}

	cam->last_frame_ts = frame_start_ts;
	cam->last_seq = seq;

	/* Push to the appropriate debug output based on frame type */
	int sink_index = slam_tracking_frame ? WMR_DEBUG_SINK_SLAM : WMR_DEBUG_SINK_CONTROLLER;
	if (u_sink_debug_is_active(&cam->debug_sinks[sink_index])) {
		u_sink_debug_push_frame(&cam->debug_sinks[sink_index], xf);
	}

	// Push to sinks
	if (slam_tracking_frame) {
		DRV_TRACE_IDENT(push_to_sinks);

		// Tracking frames usually come at ~30fps
		struct xrt_frame *frames[WMR_MAX_CAMERAS] = {NULL};
		for (int i = 0; i < cam->slam_cam_count; i++) {
			u_frame_create_roi(xf, cam->tcam_confs[i].roi, &frames[i]);
		}

		update_expgain(cam, frames);

		for (int i = 0; i < cam->slam_cam_count; i++) {
			xrt_sink_push_frame(cam->cam_sinks[i], frames[i]);
		}

		// Dashboard snapshot (reverb-g2, 2026-09-05): comes AFTER the real tracking sinks are
		// already pushed above, so nothing here can ever delay a frame's arrival at
		// SLAM/Basalt. Throttled (default WMR_CAMERA_SNAPSHOT_THROTTLE, ~1 fps out of this
		// thread's ~30fps SLAM-frame rate), overridable via WMR_CAMERA_SNAPSHOT_RATE_DIVISOR
		// for the passthrough viewer -- see wmr_camera_dump_snapshot_pgm()'s comment.
		if (cam->snapshot_enabled) {
			cam->snapshot_frame_count++;
			long snapshot_divisor = debug_get_num_option_wmr_camera_snapshot_rate_divisor();
			if (snapshot_divisor < 1) {
				snapshot_divisor = 1;
			}
			if ((cam->snapshot_frame_count % snapshot_divisor) == 0) {
				DRV_TRACE_IDENT(camera_dashboard_snapshot);
				for (int i = 0; i < cam->slam_cam_count; i++) {
					wmr_camera_dump_snapshot_pgm(cam, i, frames[i]);
				}
			}
		}

		for (int i = 0; i < cam->slam_cam_count; i++) {
			xrt_frame_reference(&frames[i], NULL);
		}
	} else {
		DRV_TRACE_IDENT(push_to_ctrl_sinks);

		// Controller-tracking frames carry all tcam_count cameras' image data, not just the SLAM subset.
		struct xrt_frame *frames[WMR_MAX_CAMERAS] = {NULL};
		for (int i = 0; i < cam->tcam_count; i++) {
			u_frame_create_roi(xf, cam->tcam_confs[i].roi, &frames[i]);
		}

		for (int i = 0; i < cam->tcam_count; i++) {
			if (cam->ctrl_cam_sinks[i] != NULL) {
				xrt_sink_push_frame(cam->ctrl_cam_sinks[i], frames[i]);
			}
		}

		// Dashboard/passthrough snapshot, controller-frametype half (reverb-g2, 2026-09-05):
		// this is the ~60fps half of the raw ~90fps combined rate that the SLAM branch above
		// never sees (frametype dispatch is mutually exclusive per frame). Comes AFTER the
		// real ctrl_cam_sinks push above, same non-delaying ordering as the SLAM branch's
		// snapshot. Shares WMR_CAMERA_SNAPSHOT_RATE_DIVISOR with the SLAM path so one knob
		// controls the combined dump rate; own counter because the two frame types arrive at
		// different cadences and must not share a modulo phase.
		if (cam->snapshot_enabled) {
			cam->ctrl_snapshot_frame_count++;
			long ctrl_snapshot_divisor = debug_get_num_option_wmr_camera_snapshot_rate_divisor();
			if (ctrl_snapshot_divisor < 1) {
				ctrl_snapshot_divisor = 1;
			}
			if ((cam->ctrl_snapshot_frame_count % ctrl_snapshot_divisor) == 0) {
				DRV_TRACE_IDENT(camera_dashboard_snapshot_ctrl);
				for (int i = 0; i < cam->tcam_count; i++) {
					wmr_camera_dump_snapshot_pgm(cam, i, frames[i]);
				}
			}
		}

		for (int i = 0; i < cam->tcam_count; i++) {
			xrt_frame_reference(&frames[i], NULL);
		}
	}

drop_frame:
	xrt_frame_reference(&xf, NULL);

out:
	libusb_submit_transfer(xfer);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct wmr_camera *
wmr_camera_open(struct wmr_camera_open_config *config)
{
	DRV_TRACE_MARKER();

	struct wmr_camera *cam = calloc(1, sizeof(struct wmr_camera));
	int res;
	int i;

	cam->tcam_count = config->tcam_count;
	cam->slam_cam_count = config->slam_cam_count;
	cam->log_level = config->log_level;
	cam->snapshot_enabled = debug_get_bool_option_wmr_camera_snapshot();

	for (int i = 0; i < cam->tcam_count; i++) {
		cam->tcam_confs[i] = *config->tcam_confs[i];
		cam->cam_sinks[i] = config->tcam_sinks[i];
		cam->ctrl_cam_sinks[i] = config->ctrl_cam_sinks != NULL ? config->ctrl_cam_sinks[i] : NULL;
	}

	if (os_thread_helper_init(&cam->usb_thread) != 0) {
		WMR_CAM_ERROR(cam, "Failed to initialise threading");
		wmr_camera_free(cam);
		return NULL;
	}

	res = libusb_init(&cam->ctx);
	if (res < 0) {
		goto fail;
	}

	struct xrt_prober_device *dev_holo = config->dev_holo;
	cam->dev = libusb_open_device_with_vid_pid(cam->ctx, dev_holo->vendor_id, dev_holo->product_id);
	if (cam->dev == NULL) {
		goto fail;
	}

	res = libusb_claim_interface(cam->dev, 3);
	if (res < 0) {
		goto fail;
	}

	cam->usb_complete = 0;
	if (os_thread_helper_start(&cam->usb_thread, wmr_cam_usb_thread, cam) != 0) {
		WMR_CAM_ERROR(cam, "Failed to start camera USB thread");
		goto fail;
	}

	for (i = 0; i < NUM_XFERS; i++) {
		cam->xfers[i] = libusb_alloc_transfer(0);
		if (cam->xfers[i] == NULL) {
			WMR_CAM_ERROR(cam, "Failed to allocate transfer %d", i);
			res = LIBUSB_ERROR_NO_MEM;
			goto fail;
		}
	}

	bool enable_aeg = debug_get_bool_option_wmr_autoexposure();
	int frame_delay = 3; // WMR takes about three frames until the cmd changes the image
	cam->unify_expgains = debug_get_bool_option_wmr_unify_expgain();

	for (int i = 0; i < cam->tcam_count; i++) {
		struct wmr_camera_expgain *ceg = &cam->ceg[i];
		ceg->manual_control = false;
		ceg->last_exposure = DEFAULT_EXPOSURE;
		ceg->exposure = DEFAULT_EXPOSURE;
		ceg->last_gain = DEFAULT_GAIN;
		ceg->gain = DEFAULT_GAIN;
		ceg->exposure_ui.val = &ceg->exposure;
		ceg->exposure_ui.max = WMR_MAX_EXPOSURE;
		ceg->exposure_ui.min = WMR_MIN_EXPOSURE;
		ceg->exposure_ui.step = 25;
		ceg->aeg = u_autoexpgain_create(U_AEG_STRATEGY_TRACKING, enable_aeg, frame_delay);
	}

	/*
	 * Exposure/gain for the controller-tracking frames, which is a SEPARATE set of hardware
	 * slots from the SLAM ones set in update_expgain(). Without this the controller frames
	 * keep whatever the sensor defaults to, which on a Reverb G2 is a completely black image
	 * even with the controllers' LEDs plainly lit and clearly visible in the SLAM stream.
	 *
	 * The slot mapping is the uncertain part. thaytan's branch has this as `camera_id + 2`,
	 * which is right for a two-camera headset (slots 0-1 SLAM, 2-3 controller) but would land
	 * on real SLAM cameras on the four-camera G2. Generalised here to + tcam_count, which
	 * reduces to thaytan's constant in the two-camera case. NOTE that his version of this loop
	 * is `for (i = cam->tcam_count; i < cam->tcam_count; i++)`, which never executes, so the
	 * mapping has almost certainly never run on any hardware -- treat it as a hypothesis under
	 * test, not a port. Acceptance is visual and immediate: "Controller Tracking Streams" in
	 * the debug GUI must stop being black and show the LED points, and "SLAM Tracking Streams"
	 * must be unchanged. If the SLAM view degrades, the mapping is wrong: revert.
	 */
	if (debug_get_bool_option_wmr_constellation_controllers()) {
		uint16_t ctrl_exposure = (uint16_t)debug_get_num_option_wmr_ctrl_exposure();
		uint8_t ctrl_gain = (uint8_t)debug_get_num_option_wmr_ctrl_gain();
		if (ctrl_exposure != DEFAULT_CTRL_EXPOSURE || ctrl_gain != DEFAULT_CTRL_GAIN) {
			// Loud on purpose: an exposure A/B is only valid if the log proves which
			// values actually ran (the 2026-08-11 experiment above showed values can
			// read back clamped by the hardware).
			WMR_CAM_INFO(cam, "Controller-tracking exposure/gain OVERRIDE: %u/%u (defaults %u/%u)",
			             ctrl_exposure, ctrl_gain, DEFAULT_CTRL_EXPOSURE, DEFAULT_CTRL_GAIN);
		}
		for (int i = 0; i < cam->tcam_count; i++) {
			const struct wmr_camera_config *config = &cam->tcam_confs[i];
			bool status = wmr_camera_set_ctrl_exposure_gain(cam, config->location, ctrl_exposure,
			                                               ctrl_gain);
			if (status != 0) {
				WMR_CAM_ERROR(cam, "Failed to set controller-tracking exposure and gain for camera %d",
				              i);
			}
		}
		// Same pair for every camera (the loop above always sends the one ctrl_exposure/ctrl_gain
		// pair), so remember it once for the dashboard snapshot (reverb-g2, 2026-09-05) rather
		// than per camera -- see wmr_camera_write_expgain_snapshot().
		cam->ctrl_exposure = ctrl_exposure;
		cam->ctrl_gain = ctrl_gain;
		cam->ctrl_expgain_set = true;
	}

	u_sink_debug_init(&cam->debug_sinks[WMR_DEBUG_SINK_SLAM]);
	u_sink_debug_init(&cam->debug_sinks[WMR_DEBUG_SINK_CONTROLLER]);
	u_var_add_root(cam, "WMR Camera", true);
	u_var_add_log_level(cam, &cam->log_level, "Log level");

	u_var_add_gui_header_begin(cam, NULL, "Camera Streams");
	u_var_add_sink_debug(cam, &cam->debug_sinks[WMR_DEBUG_SINK_SLAM], "SLAM Tracking Streams");
	u_var_add_sink_debug(cam, &cam->debug_sinks[WMR_DEBUG_SINK_CONTROLLER], "Controller Tracking Streams");
	u_var_add_gui_header_end(cam, NULL, NULL);

	// Real frame drop-rate (reverb-g2, 2026-09-05) -- see img_xfer_cb()'s seq_delta comment.
	u_var_add_u32(cam, &cam->dropped_frame_count, "Dropped frames (hardware seq gaps)");

	u_var_add_gui_header_begin(cam, NULL, "Exposure and gain control");
	u_var_add_bool(cam, &cam->unify_expgains, "Use same values");

	for (int i = 0; i < cam->tcam_count; i++) {
		struct wmr_camera_expgain *ceg = &cam->ceg[i];
		char label[256] = {0};

		(void)snprintf(label, sizeof(label), "Control for camera %d", i);
		u_var_add_gui_header_begin(cam, NULL, label);

		(void)snprintf(label, sizeof(label), "[%d] Manual exposure and gain control", i);
		u_var_add_bool(cam, &ceg->manual_control, label);

		(void)snprintf(label, sizeof(label), "[%d] Exposure (usec)", i);
		u_var_add_draggable_u16(cam, &ceg->exposure_ui, label);

		(void)snprintf(label, sizeof(label), "[%d] Gain", i);
		u_var_add_u8(cam, &ceg->gain, label);

		(void)snprintf(label, sizeof(label), "[%d] ", i);
		u_autoexpgain_add_vars(ceg->aeg, cam, label);

		u_var_add_gui_header_end(cam, NULL, NULL);
	}

	u_var_add_gui_header_end(cam, NULL, "Auto exposure and gain control END");

	return cam;

fail:
	WMR_CAM_ERROR(cam, "Failed to open camera: %s", libusb_error_name(res));
	wmr_camera_free(cam);
	return NULL;
}

void
wmr_camera_free(struct wmr_camera *cam)
{
	DRV_TRACE_MARKER();

	// Stop the camera.
	wmr_camera_stop(cam);

	if (cam->ctx != NULL) {
		int i;

		os_thread_helper_lock(&cam->usb_thread);
		cam->usb_complete = 1;
		os_thread_helper_unlock(&cam->usb_thread);

		if (cam->dev != NULL) {
			libusb_close(cam->dev);
		}

		os_thread_helper_destroy(&cam->usb_thread);

		for (i = 0; i < NUM_XFERS; i++) {
			if (cam->xfers[i] == NULL) {
				continue;
			}

			libusb_free_transfer(cam->xfers[i]);
			cam->xfers[i] = NULL;
		}

		libusb_exit(cam->ctx);
		cam->ctx = NULL;
	}

	// Tidy the variable tracking.
	u_var_remove_root(cam);
	u_sink_debug_destroy(&cam->debug_sinks[WMR_DEBUG_SINK_SLAM]);
	u_sink_debug_destroy(&cam->debug_sinks[WMR_DEBUG_SINK_CONTROLLER]);

	free(cam);
}

bool
wmr_camera_start(struct wmr_camera *cam)
{
	DRV_TRACE_MARKER();

	int res = 0;

	if (!compute_frame_size(cam)) {
		WMR_CAM_WARN(cam, "Invalid config or no head tracking cameras found");
		goto fail;
	}

	res = set_active(cam, false);
	if (res < 0) {
		goto fail;
	}

	res = set_active(cam, true);
	if (res < 0) {
		goto fail;
	}

	res = update_expgain(cam, NULL);
	if (res < 0) {
		goto fail;
	}

	for (int i = 0; i < NUM_XFERS; i++) {
		uint8_t *recv_buf = malloc(cam->xfer_size);

		libusb_fill_bulk_transfer(cam->xfers[i], cam->dev, LIBUSB_ENDPOINT_IN | 5, recv_buf, cam->xfer_size,
		                          img_xfer_cb, cam, 0);
		cam->xfers[i]->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

		res = libusb_submit_transfer(cam->xfers[i]);
		if (res < 0) {
			WMR_CAM_ERROR(cam, "Failed to submit transfer %d", i);
			goto fail;
		}
	}

	// All transfers submitted successfully and the USB thread is running: mark the camera as
	// running so wmr_camera_stop() actually performs teardown (join the USB thread, cancel
	// transfers) instead of early-returning on its `!cam->running` guard.
	cam->running = true;

	WMR_CAM_INFO(cam, "WMR camera started");

	return true;


fail:
	if (res < 0) {
		WMR_CAM_ERROR(cam, "Error starting camera input: %s", libusb_error_name(res));
	}

	wmr_camera_stop(cam);

	return false;
}

bool
wmr_camera_stop(struct wmr_camera *cam)
{
	DRV_TRACE_MARKER();

	int res;
	int i;

	if (!cam->running) {
		return true;
	}
	cam->running = false;

	for (i = 0; i < NUM_XFERS; i++) {
		if (cam->xfers[i] != NULL) {
			libusb_cancel_transfer(cam->xfers[i]);
		}
	}

	// Join the USB thread before any other frame-node's teardown can proceed --
	// otherwise an in-flight callback can still deliver a frame into a tracker
	// that is concurrently being destroyed (SIGSEGV in pop_pose()).
	os_thread_helper_stop_and_wait(&cam->usb_thread);

	res = set_active(cam, false);
	if (res < 0) {
		goto fail;
	}

	WMR_CAM_INFO(cam, "WMR camera stopped");

	return true;


fail:
	if (res < 0) {
		WMR_CAM_ERROR(cam, "Error stopping camera input: %s", libusb_error_name(res));
	}

	return false;
}

//! Throttle for wmr_camera_write_expgain_snapshot(), in update_expgain() calls (~30 Hz, one per
//! SLAM frame -- see the "Tracking frames usually come at ~30fps" comment in img_xfer_cb()).
//! 30 gives roughly 1 fps, matching WMR_CAMERA_SNAPSHOT_THROTTLE's cadence above.
#define WMR_CAMERA_EXPGAIN_SNAPSHOT_THROTTLE 30

/*!
 * Live camera exposure/gain, promoted from TRACE to something dashboard-usable (reverb-g2,
 * 2026-09-05): dump the current per-camera SLAM exposure/gain (cam->ceg[]), the fixed
 * controller-tracking exposure/gain if that opt-in feature is on, and the running dropped-frame
 * tally, to a small JSON file for the web dashboard follow-up to read. Atomic (tmp file +
 * rename) and best-effort -- same pattern as wmr_hmd.c's HMD-temperature snapshot, and a failure
 * here (HOME unset, fopen failing) must never affect the driver's real job, so it's warned once
 * via a static guard rather than every call.
 */
static void
wmr_camera_write_expgain_snapshot(struct wmr_camera *cam)
{
	static bool warned_snapshot_failure = false;
	const char *home = getenv("HOME");
	if (home == NULL) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Exposure/gain snapshot: HOME not set, skipping dashboard snapshot file");
			warned_snapshot_failure = true;
		}
		return;
	}

	char final_path[PATH_MAX];
	char tmp_path[PATH_MAX];
	snprintf(final_path, sizeof(final_path), "%s/vr/camera-expgain.json", home);
	snprintf(tmp_path, sizeof(tmp_path), "%s/vr/camera-expgain.json.tmp", home);

	FILE *snap = fopen(tmp_path, "w");
	if (snap == NULL) {
		if (!warned_snapshot_failure) {
			WMR_CAM_WARN(cam, "Exposure/gain snapshot: fopen(%s) failed: %s", tmp_path, strerror(errno));
			warned_snapshot_failure = true;
		}
		return;
	}

	fprintf(snap, "{\n");
	for (int i = 0; i < cam->tcam_count; i++) {
		fprintf(snap, "  \"cam%d\": {\"exposure_us\": %u, \"gain\": %u},\n", i, cam->ceg[i].exposure,
		        cam->ceg[i].gain);
	}
	if (cam->ctrl_expgain_set) {
		fprintf(snap, "  \"controller_tracking\": {\"exposure_us\": %u, \"gain\": %u},\n", cam->ctrl_exposure,
		        cam->ctrl_gain);
	}
	fprintf(snap, "  \"dropped_frames\": %u,\n", cam->dropped_frame_count);
	fprintf(snap, "  \"ts\": %lld\n", (long long)time(NULL));
	fprintf(snap, "}\n");
	fclose(snap);

	if (rename(tmp_path, final_path) != 0 && !warned_snapshot_failure) {
		WMR_CAM_WARN(cam, "Exposure/gain snapshot: rename(%s -> %s) failed: %s", tmp_path, final_path,
		             strerror(errno));
		warned_snapshot_failure = true;
	}
}

static int
update_expgain(struct wmr_camera *cam, struct xrt_frame **frames)
{
	int res = 0;
	for (int i = 0; i < cam->tcam_count; i++) {
		const struct wmr_camera_config *config = &cam->tcam_confs[i];

		struct wmr_camera_expgain *ceg = &cam->ceg[i];

		if (!ceg->manual_control && frames != NULL && frames[i] != NULL) {
			if (!cam->unify_expgains || i == 0) {
				u_autoexpgain_update(ceg->aeg, frames[i]);
				ceg->exposure = (uint16_t)u_autoexpgain_get_exposure(ceg->aeg);
				ceg->gain = (uint8_t)u_autoexpgain_get_gain(ceg->aeg);
			} else {
				ceg->exposure = cam->ceg[0].exposure;
				ceg->gain = cam->ceg[0].gain;
			}
		}

		if (ceg->last_exposure == ceg->exposure && ceg->last_gain == ceg->gain) {
			continue;
		}
		ceg->last_exposure = ceg->exposure;
		ceg->last_gain = ceg->gain;

		bool status = wmr_camera_set_exposure_gain(cam, config->location, ceg->exposure, ceg->gain);
		if (status != 0) {
			WMR_CAM_ERROR(cam, "Failed to set exposure and gain for camera %d", i);
		}
		res |= status;
	}

	// WMR_CTRL_EXPOSURE_FOLLOW_SLAM (see the option's comment above): only fires a USB control
	// transfer when camera 0's SLAM-autoexposed value actually moved, same change-detection
	// pattern as the per-camera loop above -- autoexposure holds steady under stable lighting,
	// so this is rare in practice, not a per-frame cost.
	if (cam->ctrl_expgain_set && debug_get_bool_option_wmr_ctrl_exposure_follow_slam()) {
		struct wmr_camera_expgain *slam0 = &cam->ceg[0];
		if (cam->ctrl_exposure != slam0->exposure || cam->ctrl_gain != slam0->gain) {
			for (int i = 0; i < cam->tcam_count; i++) {
				const struct wmr_camera_config *config = &cam->tcam_confs[i];
				bool status =
				    wmr_camera_set_ctrl_exposure_gain(cam, config->location, slam0->exposure, slam0->gain);
				if (status != 0) {
					WMR_CAM_ERROR(cam, "Failed to follow SLAM exposure/gain for controller camera %d",
					              i);
				}
			}
			cam->ctrl_exposure = slam0->exposure;
			cam->ctrl_gain = slam0->gain;
		}
	}

	cam->expgain_snapshot_count++;
	if ((cam->expgain_snapshot_count % WMR_CAMERA_EXPGAIN_SNAPSHOT_THROTTLE) == 0) {
		wmr_camera_write_expgain_snapshot(cam);
	}

	return res;
}

//! Same command, but addressing the controller-tracking exposure slots instead of the SLAM ones.
static int
wmr_camera_set_ctrl_exposure_gain(struct wmr_camera *cam, uint8_t camera_id, uint16_t exposure, uint8_t gain)
{
	// EXPERIMENT 2026-08-11: +tcam_count (the generalised mapping) sends successfully but the
	// frametype-2 exposure readback stays 0 regardless. Trying thaytan's original hardcoded
	// +2 literally instead of scaling by tcam_count, since this hardware's real camera
	// location IDs are 0,1,4,5 (not contiguous 0-3) -- a flat +2 lands on 2,3,6,7, untested.
	return wmr_camera_set_exposure_gain(cam, camera_id + 2, exposure, gain);
}

int
wmr_camera_set_exposure_gain(struct wmr_camera *cam, uint8_t camera_id, uint16_t exposure, uint8_t gain)
{
	DRV_TRACE_MARKER();

	WMR_CAM_TRACE(cam, "Setting camera %d exposure %u gain %u", camera_id, exposure, gain);
	struct wmr_camera_gain_cmd cmd = {
	    .magic = __cpu_to_le32(WMR_MAGIC),
	    .len = __cpu_to_le32(sizeof(struct wmr_camera_gain_cmd)),
	    .cmd = __cpu_to_le16(WMR_CAMERA_CMD_GAIN),
	    .camera_id = __cpu_to_le16(camera_id),
	    .exposure = __cpu_to_le16(exposure),
	    .gain = __cpu_to_le16(gain),
	    .camera_id2 = __cpu_to_le16(camera_id),
	};

	return send_buffer_to_device(cam, (uint8_t *)&cmd, sizeof(cmd));
}
