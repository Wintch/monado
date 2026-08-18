// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal ures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_frame_scribble.h"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <stdexcept>
#include <array>
#include <fstream>

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_optimize.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"


#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

extern "C" void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo);

extern "C" void
constellation_tracker_camera_destroy(t_blob_sink *tbs);

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);

extern "C" void
constellation_tracker_node_break_apart(xrt_frame_node *node);

extern "C" void
constellation_tracker_node_destroy(xrt_frame_node *node);


namespace xrt::tracking::constellation {

namespace os = xrt::auxiliary::os;

// Forward-declares
struct Camera;
struct CameraMosaic;
struct ConstellationTracker;
struct DataRecorder;
struct Device;

struct FoundDevicePose
{
	xrt_pose Tcv_cam_device XRT_POSE_IDENTITY;
	float average_blob_brightness;
};

struct DeviceState
{
	//! The ID of the device.
	t_constellation_device_id_t device_id{XRT_CONSTELLATION_INVALID_DEVICE_ID};

	//! The "predicted" pose, which is the pose the device expects itself to be at at the time of the blobservation.
	std::optional<xrt_pose> Txr_world_device_prior{std::nullopt};

	//! The final found pose of the device in this specific sample.
	std::optional<FoundDevicePose> found_pose{std::nullopt};

	//! Whether the device needs to have the slow processing thread run over it
	bool needs_slow_processing{false};
};

struct CameraSample
{
public: // Fields
	t_blobwatch *source{nullptr};
	uint64_t id{};
	int64_t timestamp_ns{};
	t_blob blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME]{};
	uint32_t blob_count{};

	//! The camera's position in the constellation tracker's tracking origin
	std::optional<xrt_pose> Txr_world_cam{std::nullopt};

	std::array<DeviceState, XRT_CONSTELLATION_MAX_DEVICES> device_states{};
	uint32_t device_count{};

	uint32_t mosaic_index;
	uint32_t camera_index;

public: // Methods
	std::optional<DeviceState *>
	getDeviceState(t_constellation_device_id_t device_id);

	DeviceState &
	putDeviceState(t_constellation_device_id_t device_id);

	CameraSample(t_blob_observation &blobservation, Camera *camera);

	CameraSample() = default;

	void
	markMatchingBlobs(ConstellationTracker *ct,
	                  t_constellation_tracker_led_model &led_model,
	                  t_constellation_device_id_t device_id,
	                  pose_metrics_blob_match_info &blob_match_info);

	t_blob_observation
	toBlobObservation() const &
	{
		t_blob_observation obs = {
		    .source = this->source,
		    .id = this->id,
		    .timestamp_ns = this->timestamp_ns,
		    .blobs = const_cast<t_blob *>(this->blobs),
		    .num_blobs = this->blob_count,
		};

		return obs;
	}

	t_blob_observation
	toBlobObservation() && = delete;
	t_blob_observation
	toBlobObservation() const && = delete;
};

struct CameraScribbleSettings
{
public: // Fields
	//! Whether to draw the blobs in the sample
	bool draw_blobs{true};
	//! Whether to draw the raw blob IDs
	bool draw_blob_ids{false};
	//! Whether to draw the device IDs on the blobs where possible
	bool draw_blob_device_ids{true};
	//! Whether to draw the LED IDs on the blobs where possible
	bool draw_blob_led_ids{false};

	//! Whether to draw the prior device pose in the sample, if it exists.
	bool draw_prior{false};
	//! Whether to draw the final device pose in the sample, if it exists.
	bool draw_found{false};

public: // Methods
	void
	setupDebugTracking(void *root);
};

struct Camera
{
public: // Fields
	t_blob_sink base = {
	    .push_blobs = constellation_tracker_camera_push_blobs,
	    .destroy = constellation_tracker_camera_destroy,
	};

	//! The owner tracker, so we can retrieve it from the blob sink callback
	ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	t_camera_calibration calibration;

	camera_model model;

	CameraScribbleSettings scribble_settings{};

	//! The index in the mosaic
	size_t index;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	os_thread_helper slow_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};

		/*!
		 * Deep-search backoff, keyed by device id: consecutive deep-search
		 * failures and the earliest monotonic time the next deep search may
		 * run. Without it, a device with no strong match (cold start, long
		 * occlusion, dim LEDs) runs the full combinatorial search to
		 * completion on every frame forever -- measured on a 6-core machine
		 * eating ~2 cores across the 4 slow threads while the solution rate
		 * collapsed to one fix every ~3s. Only touched by the slow
		 * processing thread (single-threaded mode included), so unlocked.
		 */
		struct DeepBackoff
		{
			int consecutive_failures{0};
			int64_t next_attempt_ns{0};
		};
		std::map<int, DeepBackoff> deep_backoff{};

		/*!
		 * Lost-controller search decimation state, keyed by device id
		 * (WMR_CONSTELLATION_LOST_SEARCH_DIV, docs/40 "controller-present gate", T197).
		 * @c frame_count counts eligible (needs_slow_processing) frames seen while the
		 * device has not matched recently; @c skip_this_sample is the decimation
		 * decision for the current sample, computed once on the i==0 (shallow) pass in
		 * processSampleSlow and re-read on the i==1 (deep) pass so both agree. Only
		 * touched by the slow processing thread, so unlocked (same rationale as
		 * deep_backoff above).
		 */
		struct LostSearchDecimation
		{
			uint32_t frame_count{0};
			bool skip_this_sample{false};
		};
		std::map<int, LostSearchDecimation> lost_search_decimation{};

		/*!
		 * Rate limiter for the blob-count/swamping guard's summary log line
		 * (WMR_CONSTELLATION_MAX_BLOBS). Per-camera so concurrent slow-processing
		 * threads on different cameras never race the same counter.
		 */
		struct SwampedBlobsLog
		{
			uint64_t skipped_since_log{0};
			int64_t next_log_ns{0};
		};
		SwampedBlobsLog swamped_blobs_log{};
	} slow_processing_thread_data;

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	os_thread_helper fast_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

public: // Methods (t_constellation_tracker.cpp)
	static Camera *
	Get(t_blob_sink *tbs)
	{
		return container_of(tbs, Camera, base);
	}

	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr,
	       size_t index);

	~Camera();

	// Delete all copy/move ctors, since the pointers for `base` need to be stable
	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera &
	operator=(const Camera &) = delete;
	Camera &
	operator=(Camera &&) = delete;

	std::optional<xrt_pose>
	getWorldPose(timepoint_ns when_ns);

	void
	deferSampleToSlowThread(CameraSample &sample);

	//! Fast matching based on prior pose
	bool
	tryDevicePose(std::unique_ptr<Device> &device,
	              CameraSample &sample,
	              DeviceState &device_state,
	              xrt_pose &Tcv_cam_world,
	              std::optional<xrt_pose> &Tcv_world_device_prior,
	              xrt_pose &Tcv_world_device_candidate,
	              const pose_metrics_trusted_orientation *trusted_orientation = nullptr);

	//! Continuity-based recovery: refines whatever blobs are still labeled for @p device from a
	//! previous frame via RANSAC-PnP. @p Tcv_world_device_seed_override, when non-null (reverb-g2
	//! T215, "assignment-prior SEEDING"), replaces BOTH the PnP solver's initial guess AND the
	//! prior window @ref pose_metrics_evaluate_pose_with_prior validates the refined result
	//! against -- normally both of those come from @p Tcv_world_device_prior (or identity). @p
	//! trusted_orientation, when non-null, is threaded through to the pose_metrics evaluation (and
	//! to @ref pushPose) exactly like @ref tryDevicePose already does; the ordinary call site
	//! (processSampleFast) passes neither and is therefore byte-identical to before this feature
	//! existed.
	bool
	tryDeviceBlobRecovery(std::unique_ptr<Device> &device,
	                      CameraSample &sample,
	                      DeviceState &device_state,
	                      xrt_pose &Tcv_cam_world,
	                      std::optional<xrt_pose> &Tcv_world_device_prior,
	                      const xrt_pose *Tcv_world_device_seed_override = nullptr,
	                      const pose_metrics_trusted_orientation *trusted_orientation = nullptr);

	//! Assignment-prior SEEDING (reverb-g2 T215, WMR_CONSTELLATION_SEED_PRIOR, default off):
	//! composes a candidate WORLD pose from the device's TRUSTED fusion orientation (fresh, not
	//! subject to the yaw-ghost problem this whole feature exists to route around) plus a POSITION
	//! taken from an existing prior (@p Tcv_world_device_last_known preferred, @p
	//! Tcv_world_device_prior as fallback) and tries it via the two existing fast-recovery
	//! mechanisms in turn -- @ref tryDeviceBlobRecovery (PnP-refined, if continuity has left this
	//! device's blobs labeled) then @ref tryDevicePose (pure reprojection-derived assignment
	//! otherwise). No-op (returns false immediately) unless the option is on AND @p
	//! trusted_orientation is non-null, i.e. unless @ref getTrustedOrientation already found a
	//! locked, trusted heading for this device this frame -- see that function's own gating.
	bool
	trySeededRecovery(std::unique_ptr<Device> &device,
	                  CameraSample &sample,
	                  DeviceState &device_state,
	                  xrt_pose &Tcv_cam_world,
	                  std::optional<xrt_pose> &Tcv_world_device_prior,
	                  std::optional<xrt_pose> &Tcv_world_device_last_known,
	                  const pose_metrics_trusted_orientation *trusted_orientation);

	void
	processSampleSlow(CameraSample &sample);

	//! Returns whether a slow search is needed
	bool
	processSampleFast(CameraSample &sample);

	void
	pushPose(CameraSample &camera_sample,
	         DeviceState &device_state,
	         std::unique_ptr<Device> &device,
	         pose_metrics &score,
	         xrt_pose &Tcv_cam_device,
	         bool was_optimized,
	         const pose_metrics_trusted_orientation *trusted_orientation = nullptr);

	//! Queries @p device's tracking_source (if any) for a trusted absolute orientation (@ref
	//! t_constellation_tracker_tracking_source::get_trusted_orientation), and if one is
	//! available, converts it into THIS camera's frame (Tcv_cam_device convention, matching the
	//! candidates pose_metrics evaluates) plus a camera-frame up_vector to isolate yaw with --
	//! the same conversion tryDevicePose's own Tcv_cam_device_prior already does, and the same
	//! "world up as seen from this camera" get_pose_gravity_vector already computes for
	//! CS_FLAG_MATCH_GRAVITY. Returns std::nullopt if the device has no such hook, or it has
	//! nothing trustworthy to report right now.
	std::optional<pose_metrics_trusted_orientation>
	getTrustedOrientation(std::unique_ptr<Device> &device, xrt_pose &Tcv_world_cam, int64_t when_ns);

	//! The WORLD-frame (OpenCV convention, i.e. Tcv_world_*, matching Tcv_world_device_predicted/
	//! Tcv_world_device_last_known's own convention) half of @ref getTrustedOrientation's own
	//! conversion chain, factored out so @ref trySeededRecovery can build a WORLD candidate pose
	//! from the trusted orientation without duplicating the raw tracking-source callback query and
	//! its yaw_threshold_rad validity check. @ref getTrustedOrientation itself is implemented in
	//! terms of this helper (same behavior as before this function existed, just factored). Returns
	//! std::nullopt under the identical conditions @ref getTrustedOrientation would.
	std::optional<xrt_pose>
	getTrustedWorldPose(std::unique_ptr<Device> &device, int64_t when_ns, float *out_yaw_threshold_rad);

public: // Methods (constellation_debug_scribble.cpp)
	void
	debugScribbleSample(CameraSample &sample, bool fast);
};

struct CameraMosaic
{
public: // Fields
	std::vector<std::unique_ptr<Camera>> cameras;
	//! The index of this mosaic in the tracker.
	size_t index;

	t_constellation_tracker_tracking_source *tracking_origin;

public: // Methods
	CameraMosaic(ConstellationTracker *tracker,
	             const t_constellation_tracker_camera_mosaic &mosaic_params,
	             size_t index);

	~CameraMosaic() = default;

	std::optional<xrt_pose>
	getTrackingOriginPose(timepoint_ns when_ns);
};

struct DeviceLastPose
{
public: // Fields
	xrt_pose Txr_world_device;
	timepoint_ns timestamp_ns;

public: // Methods
	DeviceLastPose(xrt_pose Txr_world_device, timepoint_ns timestamp_ns);
};

struct Device
{
public: // Fields
	t_constellation_tracker_device_params params;
	t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	// @todo remove when clang-format is updated in CI
	// clang-format off
	t_constellation_search_model *search_model{nullptr};

	// @todo These need to be pulled from the device and put into the sample. 
	//       Right now we just hardcode them since we don't have any real sensor fusion.
	xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	xrt_vec3 prior_rot_error{MIN_ROT_ERROR, MIN_ROT_ERROR, MIN_ROT_ERROR};
	float gravity_error_rad{MIN_ROT_ERROR}; /* Gravity vector uncertainty in radians 0..M_PI */

	mutable os::Mutex data_lock;
	struct
	{
		std::optional<DeviceLastPose> last_known_pose;
	} locked_data;
	// clang-format on

public: // Methods
	Device(t_constellation_tracker_device_params *params,
	       t_constellation_tracker_device *device,
	       t_constellation_device_id_t id);

	~Device();
};

// Separate base struct with our interface implementations so that `ConstellationTrackerBase` remains a standard layout
// type and we can safely use `container_of` on it.
struct ConstellationTrackerBase
{
	xrt_frame_node node = {
	    .next = nullptr,
	    .break_apart = constellation_tracker_node_break_apart,
	    .destroy = constellation_tracker_node_destroy,
	};
	xrt_tracking_origin tracking_origin = {
	    .name = "Constellation Tracker",
	    .type = XRT_TRACKING_TYPE_CONSTELLATION,
	    .initial_offset = XRT_POSE_IDENTITY,
	};
};

struct ConstellationTracker : public ConstellationTrackerBase
{
public: // Fields
	//! Whether the constellation tracker is running
	bool running = true;

	enum u_logging_level log_level = U_LOGGING_WARN;

	t_constellation_tracker_params params;

	bool single_threaded{false};

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	std::shared_mutex device_lock;
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

	std::unique_ptr<DataRecorder> data_recorder{};

#ifdef XRT_FEATURE_RERUN
	std::unique_ptr<struct RerunContext> rerun_stream{};
#endif

public: // Methods
	static ConstellationTracker *
	Get(xrt_frame_node *node)
	{
		return static_cast<ConstellationTracker *>(container_of(node, ConstellationTrackerBase, node));
	}

	static ConstellationTracker *
	Get(t_constellation_tracker *tracker)
	{
		return reinterpret_cast<ConstellationTracker *>(tracker);
	}

	ConstellationTracker(t_constellation_tracker_params *params);

	~ConstellationTracker();

	// Delete all copy/move ctors, since the pointers for `node` need to be stable
	ConstellationTracker(const ConstellationTracker &) = delete;
	ConstellationTracker(ConstellationTracker &&) = delete;
	ConstellationTracker &
	operator=(const ConstellationTracker &) = delete;
	ConstellationTracker &
	operator=(ConstellationTracker &&) = delete;

	void
	setupVariableTracking();

	t_constellation_device_id_t
	addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device);

	void
	removeDevice(t_constellation_device_id_t device_id);
};

}; // namespace xrt::tracking::constellation
