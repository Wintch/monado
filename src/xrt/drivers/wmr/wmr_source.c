// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  WMR camera and IMU data source.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup drv_wmr
 */
#include "wmr_source.h"
#include "wmr_camera.h"
#include "wmr_config.h"
#include "wmr_protocol.h"

#include "constellation/t_rift_blobwatch.h"
#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_filter_fifo.h"
#include "util/u_debug.h"
#include "util/u_sink.h"
#include "util/u_var.h"
#include "util/u_trace_marker.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frameserver.h"

#include <assert.h>
#include <stdio.h>

#define WMR_SOURCE_STR "WMR Source"

#define WMR_TRACE(w, ...) U_LOG_IFL_T(w->log_level, __VA_ARGS__)
#define WMR_DEBUG(w, ...) U_LOG_IFL_D(w->log_level, __VA_ARGS__)
#define WMR_INFO(w, ...) U_LOG_IFL_I(w->log_level, __VA_ARGS__)
#define WMR_WARN(w, ...) U_LOG_IFL_W(w->log_level, __VA_ARGS__)
#define WMR_ERROR(w, ...) U_LOG_IFL_E(w->log_level, __VA_ARGS__)
#define WMR_ASSERT(predicate, ...)                                                                                     \
	do {                                                                                                           \
		bool p = predicate;                                                                                    \
		if (!p) {                                                                                              \
			U_LOG(U_LOGGING_ERROR, __VA_ARGS__);                                                           \
			assert(false && "WMR_ASSERT failed: " #predicate);                                             \
			exit(EXIT_FAILURE);                                                                            \
		}                                                                                                      \
	} while (false);
#define WMR_ASSERT_(predicate) WMR_ASSERT(predicate, "Assertion failed " #predicate)

DEBUG_GET_ONCE_LOG_OPTION(wmr_log, "WMR_LOG", U_LOGGING_INFO)

//! Largest backwards step in the converted IMU timeline still treated as clock-offset jitter
//! rather than a real discontinuity. Measured 2026-08-12 over one session: 281 backwards events,
//! p50 3.3 ms, p99 14.7 ms, max 17.3 ms. A device-clock restart looks nothing like this -- the
//! one measured was 8.23 s -- so this threshold separates the two cleanly with room to spare.
#define IMU_JITTER_MAX_NS (20 * 1000 * 1000)

//! How far past the last accepted timestamp a floored sample is placed. Small enough to barely
//! disturb the integrator, large enough to keep strict ordering (the sample period is ~4 ms).
#define IMU_MIN_STEP_NS (250 * 1000)

/*!
 * Per-camera controller-tracking timestamp-fix sink. `base` must be the first member: this is
 * what makes `container_of` in @ref receive_ctrl_cam safe to use on an element of an array of
 * these -- unlike computing `container_of` straight from `struct wmr_source::ctrl_ts_fix_sinks`
 * (the bug this replaces), each instance here is its own object, so offsetof(base) is the same
 * fixed 0 no matter which array slot the instance lives in.
 */
struct wmr_ctrl_ts_fix_sink
{
	struct xrt_frame_sink base;
	struct wmr_source *ws;
	struct xrt_frame_sink *downstream; //!< Real blobwatch chain entry point
};

/*!
 * Handles all the data sources from the WMR driver
 *
 * @todo Currently only properly handling tracking cameras, move IMU and other sources here
 * @implements xrt_fs
 * @implements xrt_frame_node
 */
struct wmr_source
{
	struct xrt_fs xfs;
	struct xrt_frame_node node;
	enum u_logging_level log_level; //!< Log level

	struct wmr_hmd_config config;
	struct wmr_camera *camera;

	// Sinks (head tracking)
	struct xrt_frame_sink cam_sinks[WMR_MAX_CAMERAS]; //!< Intermediate sinks for camera frames
	struct xrt_imu_sink imu_sink;                     //!< Intermediate sink for IMU samples
	struct xrt_slam_sinks in_sinks;                   //!< Pointers to intermediate sinks
	struct xrt_slam_sinks out_sinks;                  //!< Pointers to downstream sinks

	// UI Sinks (head tracking)
	struct u_sink_debug ui_cam_sinks[WMR_MAX_CAMERAS]; //!< Sink to display camera frames in UI
	struct m_ff_vec3_f32 *gyro_ff;                     //!< Queue of gyroscope data to display in UI
	struct m_ff_vec3_f32 *accel_ff;                    //!< Queue of accelerometer data to display in UI

	// Controller-tracking (constellation) frames: blob detection only for now, debug-only, no tracker wired yet
	struct u_sink_debug ctrl_blob_debug_sinks[WMR_MAX_CAMERAS]; //!< Sink to display detected LED blobs in UI
	// EXPERIMENT 2026-08-11: the SLAM cam_sinks path applies +cam_hw2mono (see receive_cam0..3 below) before
	// forwarding frames, but this ctrl path never did -- so constellation sample timestamps stayed in the raw
	// hardware clock while get_tracked_pose compares against os_monotonic_get_ns(), a ~44.5M ms mismatch
	// confirmed live tonight (get_tracked_pose log: position_tracked=no, delta_ms in the tens of millions).
	// This array lets receive_ctrl_cam apply the same correction before frames reach blobwatch.
	struct wmr_ctrl_ts_fix_sink ctrl_ts_fix_sinks[WMR_MAX_CAMERAS];

	bool is_running;              //!< Whether the device is streaming
	bool first_imu_received;      //!< Don't send frames until first IMU sample
	timepoint_ns last_imu_ns;     //!< Last timepoint received.
	uint64_t imu_dropped_run;     //!< IMU samples dropped in the current out-of-order burst
	uint64_t imu_dropped_total;   //!< IMU samples dropped since startup (link/clock health)
	uint64_t imu_stabilised_total; //!< IMU samples saved by flooring instead of dropping
	time_duration_ns hw2mono;     //!< Estimated offset from IMU to monotonic clock
	time_duration_ns cam_hw2mono; //!< Caches hw2mono for use in the full frame bundle

	// Clock-skew diagnostic (2026-08-17, docs/44): the converted camera stamps were proven
	// to land p50 +578 ms in the FUTURE of the query/IMU clock at the tracker layer. These
	// fields instrument the ingest itself: raw cam-vs-IMU hardware-domain skew per frame.
	timepoint_ns last_imu_hw_ns; //!< Raw (unconverted) hw timestamp of the last IMU sample
	uint32_t cam0_frames_seen;   //!< cam0 frame counter for the clockskew log cadence
};

/*
 *
 * Sinks functionality
 *
 */

// Clock-skew diagnostic (docs/44): logs, at cam0 cadence, the raw hardware-domain skew
// between this frame's stamp and the last IMU sample's stamp, and where the CONVERTED
// stamp lands relative to monotonic now (negative age = stamped in the future). First 30
// frames log unconditionally to catch any startup-burst/anchoring transient; then 1/300
// (~1 line per 10 s at 30 Hz). raw_ts must be the PRE-conversion frame timestamp.
static void
log_cam_clockskew(struct wmr_source *ws, timepoint_ns raw_ts)
{
	ws->cam0_frames_seen++;
	if (ws->cam0_frames_seen > 30 && ws->cam0_frames_seen % 300 != 0) {
		return;
	}
	timepoint_ns now_mono = (timepoint_ns)os_monotonic_get_ns();
	double cam_minus_imu_hw_ms = (double)(raw_ts - ws->last_imu_hw_ns) / 1e6;
	double converted_minus_now_ms = (double)((raw_ts + ws->hw2mono) - now_mono) / 1e6;
	WMR_INFO(ws, "clockskew: frame=%u cam_minus_imu_hw_ms=%.2f converted_minus_now_ms=%.2f hw2mono_ms=%.2f",
	         ws->cam0_frames_seen, cam_minus_imu_hw_ms, converted_minus_now_ms, (double)ws->hw2mono / 1e6);
}

#define DEFINE_RECEIVE_CAM(cam_id)                                                                                     \
	static void receive_cam##cam_id(struct xrt_frame_sink *sink, struct xrt_frame *xf)                             \
	{                                                                                                              \
		struct wmr_source *ws = container_of(sink, struct wmr_source, cam_sinks[cam_id]);                      \
		if (cam_id == 0) {                                                                                     \
			ws->cam_hw2mono = ws->hw2mono;                                                                 \
			log_cam_clockskew(ws, xf->timestamp);                                                          \
		}                                                                                                      \
		xf->timestamp += ws->cam_hw2mono;                                                                      \
		WMR_TRACE(ws, "cam" #cam_id " img t=%" PRId64 " source_t=%" PRId64, xf->timestamp,                     \
		          xf->source_timestamp);                                                                       \
		u_sink_debug_push_frame(&ws->ui_cam_sinks[cam_id], xf);                                                \
		if (ws->out_sinks.cams[cam_id] && ws->first_imu_received) {                                            \
			xrt_sink_push_frame(ws->out_sinks.cams[cam_id], xf);                                           \
		}                                                                                                      \
	}

DEFINE_RECEIVE_CAM(0)
DEFINE_RECEIVE_CAM(1)
DEFINE_RECEIVE_CAM(2)
DEFINE_RECEIVE_CAM(3)

//! Define a function for each WMR_MAX_CAMERAS and reference it in this array
void (*receive_cam[WMR_MAX_CAMERAS])(struct xrt_frame_sink *, struct xrt_frame *) = {
    receive_cam0, //
    receive_cam1, //
    receive_cam2, //
    receive_cam3, //
};

//! EXPERIMENT 2026-08-11: mirrors receive_cam0..3's "xf->timestamp += ws->cam_hw2mono" for the
//! controller-tracking path, which never had it. One function serves all cameras, so `ws` and the
//! real downstream sink are recovered via container_of on the per-camera wmr_ctrl_ts_fix_sink
//! instance itself (see its doc comment) -- NOT by indexing into wmr_source's array directly,
//! which was tonight's actual bug: container_of only recovers the enclosing struct correctly when
//! `sink` points at the start of the named field, and for camera index i>0 it instead pointed
//! i*sizeof(xrt_frame_sink) bytes into the array, silently reading garbage for every camera but 0.
static void
receive_ctrl_cam(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct wmr_ctrl_ts_fix_sink *self = container_of(sink, struct wmr_ctrl_ts_fix_sink, base);
	struct wmr_source *ws = self->ws;

	xf->timestamp += ws->cam_hw2mono;
	xrt_sink_push_frame(self->downstream, xf);
}

static void
receive_imu_sample(struct xrt_imu_sink *sink, struct xrt_imu_sample *s)
{
	struct wmr_source *ws = container_of(sink, struct wmr_source, imu_sink);

	// Convert hardware timestamp into monotonic clock. Update offset estimate hw2mono.
	// Note this is only done with IMU samples as they have the smallest USB transmission time.
	const float IMU_FREQ = 250.f; //!< @todo use 1000 if "average_imus" is false
	timepoint_ns now_hw = s->timestamp_ns;
	timepoint_ns now_mono = (timepoint_ns)os_monotonic_get_ns();
	timepoint_ns ts = m_clock_offset_a2b(IMU_FREQ, now_hw, now_mono, &ws->hw2mono);
	ws->last_imu_hw_ns = now_hw; // raw hw stamp, for the cam clockskew diagnostic (docs/44)

	// TRIED AND REVERTED (2026-08-12): swapping this for m_clock_windowed_skew_tracker, the
	// windowed minimum-skew estimator already in this tree (and already used by the Rift
	// driver), documented as microsecond-accurate "even in the presence of 10s of milliseconds
	// of jitter". It did exactly what it promises -- IMU samples rejected as "from the past"
	// went from ~24/s to ZERO, camera bundle drops likewise -- and the tracking got WORSE, not
	// better: static drift at 60 s went from 0.7-1.0 m to 243 m and 1002 m on two consecutive
	// runs. So the dropped samples were never what limited accuracy. Untested hypothesis for
	// why it hurts: to_local maps through a fitted offset+skew line, so a slightly wrong slope
	// scales EVERY inter-sample dt, and a systematic dt bias gets integrated twice; the simple
	// filter's offset is near-constant over short spans and preserves the true hardware dt even
	// while its absolute offset is noisier. Do not re-apply it without measuring drift, not
	// just the drop counters.
	/*
	 * Check if the timepoint does time travel, we get one or two
	 * old samples when the device has not been cleanly shut down.
	 *
	 * The comment above says "one or two". Measured on a Reverb G2 on 2026-08-12, over a
	 * 17-minute session: 17997 samples dropped here, about 7% of the whole IMU stream, in a
	 * continuous trickle plus bursts -- one of which was a sustained 8.23 s backwards jump,
	 * i.e. 8 seconds during which EVERY sample was dropped and the tracker got nothing.
	 * That burst is what turned a SLAM session that had been holding 0.8 m of drift for ten
	 * minutes into a 1000 km runaway. So this is not a shutdown curiosity on this hardware,
	 * it is a routine, load- and USB-jitter-driven condition: hw2mono is estimated from
	 * os_monotonic_get_ns() at arrival, so anything that perturbs when samples arrive
	 * perturbs the estimate, and a marginal link does exactly that.
	 *
	 * Keep dropping (the sample really is out of order), but count it and make the counting
	 * visible. The old message logged one line per dropped sample -- 18k lines of noise that
	 * hid the shape of the problem rather than showing it -- and its "diff" was computed
	 * against the raw hardware timestamp instead of the converted one, so it printed
	 * nonsense like 18446744065481226985 (an unsigned underflow) and its "last" field was
	 * the new sample's own raw value.
	 */
	//
	// Before dropping, try to save the sample. The thing that moved is the *estimated offset*,
	// not the device clock: the hardware timestamps arrive monotonic, and hw2mono is re-fitted
	// on every sample from os_monotonic_get_ns() at arrival time, so USB jitter and scheduling
	// delay push the fit around by a few milliseconds. Measured over one session: 281 backwards
	// events, p50 3.3 ms, p99 14.7 ms, max 17.3 ms -- all jitter, no real discontinuity.
	//
	// Push the sample just past the last accepted timestamp instead of throwing it away. That
	// costs one sample's worth of compressed dt and keeps the measurement; dropping loses the
	// measurement AND leaves a hole. Only the sample that lands inside the dip is floored: the
	// next hardware timestamp is a full period later, so it clears the floor on its own and the
	// timeline re-syncs to the filter immediately.
	//
	// An earlier attempt reused the offset from the last accepted sample instead of flooring.
	// It ratchets: that offset can only ever follow the filter upward, so it drifts above the
	// real one and then everything looks backwards. Measured live -- 185 dropped against 1
	// saved -- and replaced with this.
	//
	// A genuine discontinuity (the device clock restarting after a USB re-enumeration -- 8.23 s
	// was measured on 2026-08-12) is far outside IMU_JITTER_MAX_NS and still falls through to
	// the drop path below. Flooring there would freeze the timeline for the whole 8 s. That case
	// needs the tracker reset, not a timestamp trick.
	if (ws->last_imu_ns > ts && (ws->last_imu_ns - ts) < IMU_JITTER_MAX_NS) {
		timepoint_ns backwards = ws->last_imu_ns - ts;
		ts = ws->last_imu_ns + IMU_MIN_STEP_NS;
		ws->imu_stabilised_total++;
		if (ws->imu_stabilised_total % 1000 == 1) {
			WMR_INFO(ws,
			         "IMU clock-offset jitter absorbed (%" PRId64 " ns backwards, kept the sample); %" PRIu64
			         " so far this session",
			         backwards, ws->imu_stabilised_total);
		}
	}

	if (ws->last_imu_ns > ts) {
		if (ws->imu_dropped_run == 0 || ws->imu_dropped_run % 250 == 0) { // ~1 s of samples
			WMR_WARN(ws,
			         "IMU sample from the past by %" PRId64 " ns (t=%" PRId64 ", last accepted=%" PRId64
			         "); dropped %" PRIu64 " in this burst, %" PRIu64 " total this session",
			         ws->last_imu_ns - ts, ts, ws->last_imu_ns, ws->imu_dropped_run + 1,
			         ws->imu_dropped_total + 1);
		}
		ws->imu_dropped_run++;
		ws->imu_dropped_total++;
		return;
	}
	if (ws->imu_dropped_run > 0) {
		WMR_INFO(ws, "IMU clock recovered after dropping %" PRIu64 " samples (%" PRIu64 " total this session)",
		         ws->imu_dropped_run, ws->imu_dropped_total);
		ws->imu_dropped_run = 0;
	}

	ws->first_imu_received = true;
	ws->last_imu_ns = ts;
	s->timestamp_ns = ts;

	struct xrt_vec3_f64 a = s->accel_m_s2;
	struct xrt_vec3_f64 w = s->gyro_rad_secs;
	WMR_TRACE(ws, "imu t=%" PRId64 " a=(%f %f %f) w=(%f %f %f)", ts, a.x, a.y, a.z, w.x, w.y, w.z);

	// Push to debug UI
	struct xrt_vec3 gyro = {(float)w.x, (float)w.y, (float)w.z};
	struct xrt_vec3 accel = {(float)a.x, (float)a.y, (float)a.z};
	m_ff_vec3_f32_push(ws->gyro_ff, &gyro, ts);
	m_ff_vec3_f32_push(ws->accel_ff, &accel, ts);

	if (ws->out_sinks.imu) {
		xrt_sink_push_imu(ws->out_sinks.imu, s);
	}
}


/*
 *
 * Frameserver functionality
 *
 */

static inline struct wmr_source *
wmr_source_from_xfs(struct xrt_fs *xfs)
{
	struct wmr_source *ws = container_of(xfs, struct wmr_source, xfs);
	return ws;
}

static bool
wmr_source_enumerate_modes(struct xrt_fs *xfs, struct xrt_fs_mode **out_modes, uint32_t *out_count)
{
	WMR_ASSERT(false, "Not implemented");
	return false;
}

static bool
wmr_source_configure_capture(struct xrt_fs *xfs, struct xrt_fs_capture_parameters *cp)
{
	WMR_ASSERT(false, "Not implemented");
	return false;
}

static bool
wmr_source_stream_stop(struct xrt_fs *xfs)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = wmr_source_from_xfs(xfs);

	bool stopped = wmr_camera_stop(ws->camera);
	if (!stopped) {
		WMR_ERROR(ws, "Unable to stop WMR cameras");
		WMR_ASSERT_(false);
	}

	return stopped;
}

static bool
wmr_source_is_running(struct xrt_fs *xfs)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = wmr_source_from_xfs(xfs);
	return ws->is_running;
}

static bool
wmr_source_stream_start(struct xrt_fs *xfs,
                        struct xrt_frame_sink *xs,
                        enum xrt_fs_capture_type capture_type,
                        uint32_t descriptor_index)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = wmr_source_from_xfs(xfs);

	if (xs == NULL && capture_type == XRT_FS_CAPTURE_TYPE_TRACKING) {
		WMR_INFO(ws, "Starting WMR stream in tracking mode");
	} else if (xs != NULL && capture_type == XRT_FS_CAPTURE_TYPE_CALIBRATION) {
		WMR_INFO(ws, "Starting WMR stream in calibration mode, will stream only cam0 frames");
		ws->out_sinks.cam_count = 1;
		ws->out_sinks.cams[0] = xs;
	} else {
		WMR_ASSERT(false, "Unsupported stream configuration xs=%p capture_type=%d", (void *)xs, capture_type);
		return false;
	}

	bool started = wmr_camera_start(ws->camera);
	if (!started) {
		WMR_ERROR(ws, "Unable to start WMR cameras");
		WMR_ASSERT_(false);
	}

	ws->is_running = started;
	return ws->is_running;
}

static bool
wmr_source_slam_stream_start(struct xrt_fs *xfs, struct xrt_slam_sinks *sinks)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = wmr_source_from_xfs(xfs);
	if (sinks != NULL) {
		ws->out_sinks = *sinks;
	}
	return wmr_source_stream_start(xfs, NULL, XRT_FS_CAPTURE_TYPE_TRACKING, 0);
}


/*
 *
 * Frame node functionality
 *
 */

static void
wmr_source_node_break_apart(struct xrt_frame_node *node)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = container_of(node, struct wmr_source, node);
	wmr_source_stream_stop(&ws->xfs);
}

static void
wmr_source_node_destroy(struct xrt_frame_node *node)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = container_of(node, struct wmr_source, node);
	WMR_DEBUG(ws, "Destroying WMR source");
	for (int i = 0; i < ws->config.tcam_count; i++) {
		u_sink_debug_destroy(&ws->ui_cam_sinks[i]);
		u_sink_debug_destroy(&ws->ctrl_blob_debug_sinks[i]);
	}
	m_ff_vec3_f32_free(&ws->gyro_ff);
	m_ff_vec3_f32_free(&ws->accel_ff);
	u_var_remove_root(ws);
	if (ws->camera != NULL) { // It could be null if XRT_HAVE_LIBUSB is not defined
		wmr_camera_free(ws->camera);
	}
	free(ws);
}


/*
 *
 * Exported functions
 *
 */

//! Create and open the frame server for IMU/camera streaming.
struct xrt_fs *
wmr_source_create(struct xrt_frame_context *xfctx,
                  struct xrt_prober_device *dev_holo,
                  struct wmr_hmd_config cfg,
                  struct t_blob_sink **ctrl_cam_constellation_sinks)
{
	DRV_TRACE_MARKER();

	struct wmr_source *ws = U_TYPED_CALLOC(struct wmr_source);
	ws->log_level = debug_get_log_option_wmr_log();

	// Setup xrt_fs
	struct xrt_fs *xfs = &ws->xfs;
	xfs->enumerate_modes = wmr_source_enumerate_modes;
	xfs->configure_capture = wmr_source_configure_capture;
	xfs->stream_start = wmr_source_stream_start;
	xfs->slam_stream_start = wmr_source_slam_stream_start;
	xfs->stream_stop = wmr_source_stream_stop;
	xfs->is_running = wmr_source_is_running;
	(void)snprintf(xfs->name, sizeof(xfs->name), WMR_SOURCE_STR);
	(void)snprintf(xfs->product, sizeof(xfs->product), WMR_SOURCE_STR " Product");
	(void)snprintf(xfs->manufacturer, sizeof(xfs->manufacturer), WMR_SOURCE_STR " Manufacturer");
	(void)snprintf(xfs->serial, sizeof(xfs->serial), WMR_SOURCE_STR " Serial");
	xfs->source_id = *((uint64_t *)"WMR_SRC\0");

	// Setup sinks
	for (int i = 0; i < WMR_MAX_CAMERAS; i++) {
		ws->cam_sinks[i].push_frame = receive_cam[i];
	}
	ws->imu_sink.push_imu = receive_imu_sample;

	ws->in_sinks.cam_count = cfg.tcam_count;
	for (int i = 0; i < cfg.tcam_count; i++) {
		ws->in_sinks.cams[i] = &ws->cam_sinks[i];
	}
	ws->in_sinks.imu = &ws->imu_sink;

	// Controller-tracking (frametype 0x2) frames: run LED blob detection per camera and show it in the debug
	// GUI. When ctrl_cam_constellation_sinks is set (WMR_CONSTELLATION_CONTROLLERS=1), detections are also
	// forwarded to the constellation tracker; the debug panel keeps working either way.
	struct xrt_frame_sink *ctrl_cam_sinks[WMR_MAX_CAMERAS] = {NULL};
	for (int i = 0; i < cfg.tcam_count; i++) {
		u_sink_debug_init(&ws->ctrl_blob_debug_sinks[i]);

		struct t_blob_sink *downstream =
		    ctrl_cam_constellation_sinks != NULL ? ctrl_cam_constellation_sinks[i] : NULL;

		struct t_blob_sink *blob_sink = NULL;
		u_sink_blob_visualizer_create(xfctx, downstream, &ws->ctrl_blob_debug_sinks[i],
		                              cfg.tcams[i]->roi.extent.w, cfg.tcams[i]->roi.extent.h, &blob_sink);

		struct t_rift_blobwatch_params bw_params = {
		    .pixel_threshold = RIFT_BLOBWATCH_PIXEL_THRESHOLD_CV1,
		    .blob_required_threshold = RIFT_BLOBWATCH_BLOB_REQUIRED_THRESHOLD,
		    .max_match_dist = RIFT_BLOBWATCH_DEFAULT_MAX_MATCH_DIST,
		    .max_blob_width = RIFT_BLOBWATCH_DEFAULT_MAX_BLOB_WIDTH,
		};
		struct xrt_frame_sink *frame_sink = NULL;
		struct t_blobwatch *blobwatch = NULL;
		int ret = t_rift_blobwatch_create(&bw_params, xfctx, blob_sink, &frame_sink, &blobwatch);
		if (ret != 0) {
			WMR_WARN(ws, "Failed to create controller-tracking blobwatch for camera %d: %d", i, ret);
			continue;
		}

		if (!u_sink_simple_queue_create(xfctx, frame_sink, &frame_sink)) {
			WMR_WARN(ws, "Failed to create controller-tracking blobwatch queue for camera %d", i);
			continue;
		}

		// EXPERIMENT 2026-08-11: apply the same hw->mono clock correction the SLAM cam_sinks path gets
		// (see receive_cam0..3) before frames reach blobwatch, so constellation sample timestamps end up
		// in the same clock domain get_tracked_pose compares against. See the struct field comments.
		ws->ctrl_ts_fix_sinks[i].base.push_frame = receive_ctrl_cam;
		ws->ctrl_ts_fix_sinks[i].ws = ws;
		ws->ctrl_ts_fix_sinks[i].downstream = frame_sink;
		ctrl_cam_sinks[i] = &ws->ctrl_ts_fix_sinks[i].base;
	}

	struct wmr_camera_open_config options = {
	    .dev_holo = dev_holo,
	    .tcam_confs = cfg.tcams,
	    .tcam_sinks = ws->in_sinks.cams,
	    .ctrl_cam_sinks = ctrl_cam_sinks,
	    .tcam_count = cfg.tcam_count,
	    .slam_cam_count = cfg.slam_cam_count,
	    .log_level = ws->log_level,
	};

	ws->camera = wmr_camera_open(&options);
	ws->config = cfg;

	// Setup UI
	for (int i = 0; i < cfg.tcam_count; i++) {
		u_sink_debug_init(&ws->ui_cam_sinks[i]);
	}
	m_ff_vec3_f32_alloc(&ws->gyro_ff, 1000);
	m_ff_vec3_f32_alloc(&ws->accel_ff, 1000);
	u_var_add_root(ws, WMR_SOURCE_STR, false);
	u_var_add_log_level(ws, &ws->log_level, "Log Level");
	u_var_add_ro_ff_vec3_f32(ws, ws->gyro_ff, "Gyroscope");
	u_var_add_ro_ff_vec3_f32(ws, ws->accel_ff, "Accelerometer");
	for (int i = 0; i < cfg.tcam_count; i++) {
		char label[] = "Camera NNNNNNNNNNN";
		(void)snprintf(label, sizeof(label), "Camera %d", i);
		u_var_add_sink_debug(ws, &ws->ui_cam_sinks[i], label);
	}
	for (int i = 0; i < cfg.tcam_count; i++) {
		char label[] = "Controller Blob Cam NNNNNNNNNNN";
		(void)snprintf(label, sizeof(label), "Controller Blob Cam %d", i);
		u_var_add_sink_debug(ws, &ws->ctrl_blob_debug_sinks[i], label);
	}

	// Setup node
	struct xrt_frame_node *xfn = &ws->node;
	xfn->break_apart = wmr_source_node_break_apart;
	xfn->destroy = wmr_source_node_destroy;
	xrt_frame_context_add(xfctx, &ws->node);

	WMR_DEBUG(ws, "WMR Source created");

	return xfs;
}

void
wmr_source_push_imu_packet(struct xrt_fs *xfs, timepoint_ns t, struct xrt_vec3 accel, struct xrt_vec3 gyro)
{
	DRV_TRACE_MARKER();
	struct wmr_source *ws = wmr_source_from_xfs(xfs);
	struct xrt_vec3_f64 accel_f64 = {accel.x, accel.y, accel.z};
	struct xrt_vec3_f64 gyro_f64 = {gyro.x, gyro.y, gyro.z};
	struct xrt_imu_sample sample = {.timestamp_ns = t, .accel_m_s2 = accel_f64, .gyro_rad_secs = gyro_f64};
	xrt_sink_push_imu(&ws->imu_sink, &sample);
}
