// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the main logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "xrt/xrt_config_build.h"

#include "t_constellation_tracker_internal.hpp"
#include "t_constellation_tracker_dataset.hpp"

#include "os/os_time.h"
#include "util/u_time.h"
#include "util/u_thread_priority.h"

#ifdef XRT_FEATURE_RERUN
#include "constellation_tracker_rerun.hpp"
#endif

#include <cinttypes>
#include <string>


namespace xrt::tracking::constellation {

DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_OPTION(constellation_tracker_data_recorder_output, "CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT", "")

// Unconditionally present to allow warning that the feature is not enabled.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_enable_rerun, "CONSTELLATION_TRACKER_RERUN_ENABLE", false)
#ifdef XRT_FEATURE_RERUN
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_rerun_spawn, "CONSTELLATION_TRACKER_RERUN_SPAWN", true)
#endif

// Blob-count / swamping guard (reverb-g2 docs/40 "Refined fix direction"; see the long
// comment in Camera::processSampleSlow for the calibration story). Default 0 = off:
// deliberately no built-in guess at N, see that comment for why.
DEBUG_GET_ONCE_NUM_OPTION(constellation_max_blobs, "WMR_CONSTELLATION_MAX_BLOBS", 0)

// Lost-controller search decimation (reverb-g2 docs/40 "controller-present gate", T197).
// Default 0 = off. See the long comment in Camera::processSampleSlow.
DEBUG_GET_ONCE_NUM_OPTION(constellation_lost_search_div, "WMR_CONSTELLATION_LOST_SEARCH_DIV", 0)

// Assignment-prior SEEDING (reverb-g2 T215, following on from 0074/0076's reject-and-prefer
// gates on the yaw-flip ghost problem). Those gates judge candidates the blind correspondence
// search already produced; T215 measured that isn't enough on its own -- the left controller's
// yaw-symmetric LED ring means the search rarely CONSTRUCTS the true-lobe assignment at all
// (position delivered only 4.55% of samples, vs 30.3% on the right). This composes a candidate
// pose from the trusted fusion orientation plus a position from an existing prior and tries it
// directly (see Camera::trySeededRecovery), instead of waiting for the blind search to stumble
// onto it. Default off: it only ever engages when a trusted orientation is ALSO available (see
// that function's own gating), so this is a second, independent switch layered on top of
// WMR_CONSTELLATION_YAW_PRIOR_DEG's own enablement, not a replacement for it.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_seed_prior, "WMR_CONSTELLATION_SEED_PRIOR", false)

/*
 *
 * Helper functions
 *
 */

//! Gets the gravity vector of a pose, in the pose's local frame.
static void
get_pose_gravity_vector(xrt_pose &T_world_pose, xrt_vec3 &gravity)
{
	// Extract the gravity vector from the pose's orientation
	gravity = XRT_VEC3_UNIT_Y;

	xrt_quat T_pose_world_orientation;
	math_quat_invert(&T_world_pose.orientation, &T_pose_world_orientation);

	math_quat_rotate_vec3(&T_pose_world_orientation, &gravity, &gravity);
}

static uint32_t
num_blobs_for_device(CameraSample &sample, t_constellation_device_id_t device_id)
{
	uint32_t out_num_blobs = 0;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		t_blob &b = sample.blobs[i];
		if (b.matched_device_id == device_id) {
			out_num_blobs++;
		}
	}
	return out_num_blobs;
}

// Per-device, rate-limited seeded-recovery diagnostics (reverb-g2 T215, "assignment-prior
// SEEDING", Camera::trySeededRecovery below). Plain fixed-size arrays, not atomics and not a
// std::map: the SAME device can be visited concurrently by more than one camera's own
// fast-processing thread (one thread per camera, see Camera::fast_processing_thread), so these
// counters genuinely race across threads -- tolerated as a diagnostic-only benign race, same
// trade pose_metrics.c's own trusted_yaw_reject_count/trusted_yaw_deprioritize_count already
// make (see that file's comment). Fixed-size and bounds-checked rather than a std::map: a
// concurrent unlocked map insertion is a real memory-safety hazard, not just an imprecise
// counter, whereas racing a plain integer increment in a fixed slot is not.
// XRT_CONSTELLATION_MAX_DEVICES is already the hard cap this file uses elsewhere for
// per-sample device arrays (see CameraSample::putDeviceState).
static uint64_t seed_recovery_attempt_count[XRT_CONSTELLATION_MAX_DEVICES] = {};
static uint64_t seed_recovery_skip_count[XRT_CONSTELLATION_MAX_DEVICES] = {};
static uint64_t seed_recovery_success_count[XRT_CONSTELLATION_MAX_DEVICES] = {};

static uint64_t *
seed_recovery_counter_slot(uint64_t *counters, t_constellation_device_id_t device_id)
{
	static uint64_t discard; // Out-of-range device id: never expected in practice, diagnostics only.
	if (device_id < 0 || device_id >= XRT_CONSTELLATION_MAX_DEVICES) {
		return &discard;
	}
	return &counters[device_id];
}

/*
 *
 * CameraSample implementations
 *
 */

std::optional<DeviceState *>
CameraSample::getDeviceState(t_constellation_device_id_t device_id)
{
	for (uint32_t i = 0; i < this->device_count; i++) {
		if (this->device_states[i].device_id == device_id) {
			return &this->device_states[i];
		}
	}

	return std::nullopt;
}

DeviceState &
CameraSample::putDeviceState(t_constellation_device_id_t device_id)
{
	assert(this->device_count < XRT_CONSTELLATION_MAX_DEVICES);

	DeviceState &new_device_state = this->device_states[this->device_count++];
	new_device_state = {
	    .device_id = device_id,
	    .Txr_world_device_prior = std::nullopt,
	    .found_pose = std::nullopt,
	    .needs_slow_processing = false,
	};

	return new_device_state;
}


CameraSample::CameraSample(t_blob_observation &blobservation, Camera *camera)
{
	// Copy the blob observation into this sample, since we need the data to be safe.
	this->source = blobservation.source;
	this->id = blobservation.id;
	this->timestamp_ns = blobservation.timestamp_ns;
	memcpy(blobs, blobservation.blobs, sizeof(t_blob) * blobservation.num_blobs);
	this->blob_count = blobservation.num_blobs;

	// Get the camera pose
	this->Txr_world_cam = camera->getWorldPose(blobservation.timestamp_ns);

	this->device_states = {};
	this->device_count = 0;

	auto mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_THROW(mosaic, "Camera's mosaic was destroyed while we still had a sample referencing it");

	this->mosaic_index = mosaic->index;
	this->camera_index = camera->index;
}

void
CameraSample::markMatchingBlobs(ConstellationTracker *ct,
                                t_constellation_tracker_led_model &led_model,
                                t_constellation_device_id_t device_id,
                                pose_metrics_blob_match_info &blob_match_info)
{
	// First clear existing blob labels for this device
	for (uint32_t i = 0; i < this->blob_count; i++) {
		t_blob &b = this->blobs[i];

		// Skip blobs which already have an ID not belonging to this device
		if (b.matched_device_id != device_id) {
			continue;
		}

		if (b.matched_device_led_id != XRT_CONSTELLATION_INVALID_LED_ID) {
			// @todo is this needed?
			// b.prev_led_id = b.led_id;
		}

		b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	}

	// Iterate the visible LEDs and mark matching blobs with this device ID and LED ID
	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		pose_metrics_visible_led_info &led_info = blob_match_info.visible_leds[i];
		t_constellation_tracker_led *led = led_info.led;

		if (led_info.matched_blob != NULL) {
			t_blob *b = led_info.matched_blob;

			b->matched_device_led_id = led->id;
			b->matched_device_id = device_id;

			CT_DEBUG(ct, "Marking LED %d/%d at %f,%f angle %f now %d (was %d)", device_id, led->id,
			         b->center.x, b->center.y, RAD_TO_DEG(acosf(led_info.facing_dot)),
			         b->matched_device_led_id, /* b->prev_led_id */ -1);
		} else {
			CT_DEBUG(ct, "No blob for device %d LED %d @ %f,%f size %f px angle %f", device_id, led->id,
			         led_info.pos_px.x, led_info.pos_px.y, 2 * led_info.led_radius_px,
			         RAD_TO_DEG(acosf(led_info.facing_dot)));
		}
	}
}

/*
 *
 * Camera implementations
 *
 */

Camera::Camera(ConstellationTracker *tracker,
               std::weak_ptr<CameraMosaic> mosaic,
               const t_constellation_tracker_camera &camera_params,
               enum u_logging_level *log_level_ptr,
               size_t index)
    : tracker(tracker), mosaic(mosaic), calibration(camera_params.calibration), index(index)
{
	this->tracker = tracker;
	this->mosaic = mosaic;
	this->calibration = camera_params.calibration;
	this->model = {
	    .width = calibration.image_size_pixels.w,
	    .height = calibration.image_size_pixels.h,
	    .calib = {},
	};
	t_camera_model_params_from_t_camera_calibration(&this->calibration, &this->model.calib);

	this->locked_data = {
	    .Txr_origin_cam = camera_params.pose_in_origin,
	    .has_concrete_pose = camera_params.has_concrete_pose,
	};

	this->slow_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);
	this->fast_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);

	if (!this->tracker->single_threaded) {
		if (os_thread_helper_init(&this->slow_processing_thread) < 0) {
			throw std::runtime_error("Slow processing thread failed to init");
		}
		if (os_thread_helper_init(&this->fast_processing_thread) < 0) {
			throw std::runtime_error("Fast processing thread failed to init");
		}

		if (os_thread_helper_start(&this->slow_processing_thread, constellation_tracker_camera_slow_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting slow processing thread failed");
		}
		if (os_thread_helper_start(&this->fast_processing_thread, constellation_tracker_camera_fast_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting fast processing thread failed");
		}
	}

	u_sink_debug_init(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_init(&this->fast_processing_thread_data.debug_sink);
}

Camera::~Camera()
{
	if (this->slow_processing_thread.initialized) {
		os_thread_helper_destroy(&this->slow_processing_thread);
	}

	if (this->fast_processing_thread.initialized) {
		os_thread_helper_destroy(&this->fast_processing_thread);
	}

	if (this->slow_processing_thread_data.cs) {
		correspondence_search_free(this->slow_processing_thread_data.cs);
		this->slow_processing_thread_data.cs = nullptr;
	}

	if (this->fast_processing_thread_data.cs) {
		correspondence_search_free(this->fast_processing_thread_data.cs);
		this->fast_processing_thread_data.cs = nullptr;
	}

	u_sink_debug_destroy(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_destroy(&this->fast_processing_thread_data.debug_sink);
}

std::optional<xrt_pose>
Camera::getWorldPose(timepoint_ns when_ns)
{
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(
	    mosaic, "Camera's mosaic was destroyed while we still had a pointer to it, this should never happen",
	    std::nullopt);

	auto Txr_world_origin = mosaic->getTrackingOriginPose(when_ns);
	if (!Txr_world_origin.has_value()) {
		return std::nullopt;
	}

	std::unique_lock<os::Mutex> lock(this->processing_lock);

	if (!this->locked_data.has_concrete_pose) {
		return std::nullopt;
	}

	xrt_pose Txr_world_cam;
	math_pose_transform(&Txr_world_origin.value(), &this->locked_data.Txr_origin_cam, &Txr_world_cam);
	return Txr_world_cam;
}

void
Camera::deferSampleToSlowThread(CameraSample &sample)
{
	os_thread_helper_lock(&this->slow_processing_thread);
	this->slow_processing_thread_data.sample = sample;
	os_thread_helper_signal_locked(&this->slow_processing_thread);
	os_thread_helper_unlock(&this->slow_processing_thread);
}

bool
Camera::tryDevicePose(std::unique_ptr<Device> &device,
                      CameraSample &sample,
                      DeviceState &device_state,
                      xrt_pose &Tcv_cam_world,
                      std::optional<xrt_pose> &Tcv_world_device_prior,
                      xrt_pose &Tcv_world_device_candidate,
                      const pose_metrics_trusted_orientation *trusted_orientation)
{
	xrt_pose Tcv_cam_device_candidate;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_candidate, &Tcv_cam_device_candidate);

	pose_metrics score;
	if (Tcv_world_device_prior.has_value()) {
		xrt_pose Tcv_cam_device_prior;
		math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior.value(), &Tcv_cam_device_prior);

		pose_metrics_evaluate_pose_with_prior(&score, &Tcv_cam_device_candidate, false, &Tcv_cam_device_prior,
		                                      &device->prior_pos_error, &device->prior_rot_error, sample.blobs,
		                                      sample.blob_count, &device->params.led_model, device->id,
		                                      &this->model, NULL, trusted_orientation);
	} else {
		pose_metrics_evaluate_pose(&score, &Tcv_cam_device_candidate, sample.blobs, sample.blob_count,
		                           &device->params.led_model, device->id, &this->model, NULL,
		                           trusted_orientation);
	}


	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD | POSE_MATCH_LED_IDS)) {
		this->pushPose(sample,                   //
		               device_state,             //
		               device,                   //
		               score,                    //
		               Tcv_cam_device_candidate, //
		               false,                    //
		               trusted_orientation);      //
		return true;
	}

	return false;
}

bool
Camera::tryDeviceBlobRecovery(std::unique_ptr<Device> &device,
                              CameraSample &sample,
                              DeviceState &device_state,
                              xrt_pose &Tcv_cam_world,
                              std::optional<xrt_pose> &Tcv_world_device_prior,
                              const xrt_pose *Tcv_world_device_seed_override,
                              const pose_metrics_trusted_orientation *trusted_orientation)
{
	auto tracker = this->tracker;

	const uint32_t needed_blobs = 4;
	uint32_t num_blobs = 0;
	for (uint32_t index = 0; index < sample.blob_count; index++) {
		t_blob &b = sample.blobs[index];
		if (b.matched_device_id == device->id) {
			num_blobs++;

			if (num_blobs >= needed_blobs) {
				break;
			}
		}
	}
	if (num_blobs < needed_blobs) {
		return false;
	}

	// Tcv_world_device_seed_override (reverb-g2 T215, "assignment-prior SEEDING") replaces the
	// ordinary Tcv_world_device_prior for BOTH purposes below when supplied: the RANSAC-PnP
	// solver's initial guess (a better guess than the identity/prior fallback measurably matters
	// here -- the whole reason the seeded caller exists is that the ordinary guesses converge to
	// the LED ring's ghost lobe), and the prior window pose_metrics_evaluate_pose_with_prior
	// validates the refined result against just below (using the ordinary, possibly-stale or
	// still-poisoned Tcv_world_device_prior as that reference here would defeat the seed: a
	// correct, seed-derived result would get rejected for disagreeing with a wrong prior). The
	// ordinary call site (processSampleFast) never supplies this, so `have_prior_for_check` is
	// exactly `Tcv_world_device_prior.has_value()` there, byte-identical to before this parameter
	// existed.
	xrt_pose Tcv_cam_device_prior;
	bool have_prior_for_check = false;
	if (Tcv_world_device_seed_override != nullptr) {
		math_pose_transform(&Tcv_cam_world, Tcv_world_device_seed_override, &Tcv_cam_device_prior);
		have_prior_for_check = true;
	} else if (Tcv_world_device_prior.has_value()) {
		math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior.value(), &Tcv_cam_device_prior);
		have_prior_for_check = true;
	} else {
		// The RANSAC should still be able to run with an identity pose.
		Tcv_cam_device_prior = XRT_POSE_IDENTITY;
	}

	// RANSAC-PnP with the matched blobs
	xrt_pose Tcv_cam_device = Tcv_cam_device_prior;
	if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, sample.blobs, sample.blob_count,
	                     &device->params.led_model, device->id, &this->model, NULL, NULL)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs failed", (void *)this,
		         device->id, sample.blob_count);
		return false;
	}

	pose_metrics score;
	// Evaluate the pose, using the prior/seed if available
	if (have_prior_for_check) {
		// prior_must_match=true makes this cheap recovery path hard-reject any
		// candidate outside the prior window -- but the window is a fixed
		// +-10cm/30deg (MIN_POS_ERROR/MIN_ROT_ERROR, never updated anywhere)
		// applied to a prior that extrapolates with zero velocity, i.e. it
		// freezes at the last real fix. A hand that moved more than 10cm since
		// then can never use this path again and every frame falls through to
		// the expensive full search -- the mechanism behind a controller staying
		// "anchored" for seconds after occlusion. Grow the acceptance window
		// with the age of the last real sample instead: fresh prior keeps the
		// tight gate (which is what protects against the known 0.19m
		// pose-hypothesis flip), stale prior degrades smoothly toward
		// quality-only acceptance. 0.5 m/s and 60 deg/s are conservative for a
		// hand; both capped so the gate never disappears entirely.
		xrt_vec3 pos_window = device->prior_pos_error;
		xrt_vec3 rot_window = device->prior_rot_error;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);
			if (device->locked_data.last_known_pose.has_value() &&
			    sample.timestamp_ns > device->locked_data.last_known_pose->timestamp_ns) {
				float age_s =
				    (float)(sample.timestamp_ns - device->locked_data.last_known_pose->timestamp_ns) /
				    1e9f;
				float pos_grow = 0.5f * age_s;
				float rot_grow = (float)DEG_TO_RAD(60) * age_s;
				pos_window.x = fminf(pos_window.x + pos_grow, 0.75f);
				pos_window.y = fminf(pos_window.y + pos_grow, 0.75f);
				pos_window.z = fminf(pos_window.z + pos_grow, 0.75f);
				rot_window.x = fminf(rot_window.x + rot_grow, (float)DEG_TO_RAD(150));
				rot_window.y = fminf(rot_window.y + rot_grow, (float)DEG_TO_RAD(150));
				rot_window.z = fminf(rot_window.z + rot_grow, (float)DEG_TO_RAD(150));
			}
		}
		pose_metrics_evaluate_pose_with_prior(&score, &Tcv_cam_device, true, &Tcv_cam_device_prior,
		                                      &pos_window, &rot_window, sample.blobs,
		                                      sample.blob_count, &device->params.led_model, device->id,
		                                      &this->model, NULL, trusted_orientation);
	} else {
		pose_metrics_evaluate_pose(&score, &Tcv_cam_device, sample.blobs, sample.blob_count,
		                           &device->params.led_model, device->id, &this->model, NULL,
		                           trusted_orientation);
	}

	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose for device %d from %u blobs", (void *)this,
		         device->id, sample.blob_count);
		this->pushPose(sample,               //
		               device_state,         //
		               device,               //
		               score,                //
		               Tcv_cam_device,       //
		               true,                 //
		               trusted_orientation); //

		return true;
	}

	return false;
}

/*!
 * Assignment-prior SEEDING (reverb-g2 T215, docs/pruebas.jsonl -- the third layer of the
 * yaw-ghost saga, following 0074's device-side reject gate and 0076's tracker-side
 * trusted-orientation plumbing). Both of those JUDGE candidates the blind correspondence
 * search already produced; T215 measured that's not enough on its own for the left
 * controller's yaw-symmetric LED ring -- the search rarely CONSTRUCTS the true-lobe
 * assignment at all (position delivered only 4.55% of samples, vs 30.3% on the right,
 * even with 0076 filtering what it does find). This composes a candidate pose directly
 * from the trusted heading instead of waiting for the blind search to stumble onto it.
 *
 * Placement/trigger: called from Camera::processSampleFast, as the LAST fast-path attempt
 * for a device, after both ordinary tryDevicePose calls (predicted-prior, then
 * last-known-pose) have already failed this frame -- i.e. every frame the ordinary fast
 * path was about to give up and either wipe this device's blob associations or defer to
 * the (rate-limited, far less frequent) slow combinatorial search. This placement was
 * chosen over "every N frames lost" because the seeded attempt below is cheap (no
 * combinatorics -- at most one RANSAC-PnP refine plus one reprojection match, the same
 * per-frame cost tryDeviceBlobRecovery/tryDevicePose already pay), so there is no reason
 * to throttle it; running it on every fast-path miss maximises how many chances per
 * second the true lobe gets to be reconstructed, which is exactly what the measured
 * scarcity needs. It was NOT placed as "immediately after the slow thread rejects a
 * ghost" (the design doc's other suggested trigger) because that event isn't visible at
 * the call site: correspondence_search_find_one_pose reports a trusted-yaw-rejected
 * candidate identically to "found nothing at all" (found_pose == false either way) -- the
 * fast-path-miss placement fires in both of those cases anyway, so it subsumes that
 * trigger without needing new plumbing to distinguish them.
 *
 * Seed composition: orientation from the device's trusted fusion heading (@ref
 * getTrustedWorldPose, the same callback 0076 already wires up via
 * WMR_CONSTELLATION_YAW_PRIOR_DEG's enablement -- no new env for that half); position from
 * @p Tcv_world_device_last_known if available, else @p Tcv_world_device_prior. Preferring
 * last-known over the predicted prior is deliberate: a half-turn yaw-flip ghost's POSITION
 * error is small (same handful of LEDs, just reprojected under the wrong assignment), so
 * even a last_known_pose written by a recent ghost is still a usable position anchor --
 * whereas the predicted prior extrapolates m_relation_history, which (pre-0076, or for any
 * sample that slipped past both gates) could itself carry a poisoned entry forward.
 *
 * Two-attempt cascade, in order: (1) tryDeviceBlobRecovery with the seed pose overriding
 * its usual PnP initial guess -- most robust to a coarse position guess, since PnP solves
 * for pose from whatever correspondences blob continuity already labeled, using the seed
 * only to steer the (otherwise locally-convergent, and therefore genuinely ambiguous on a
 * symmetric LED ring) iterative solver toward the true lobe instead of the ghost. (2)
 * tryDevicePose with the seed pose as the candidate directly -- pure reprojection-derived
 * assignment, no PnP, covering a device with nothing labeled for it at all this frame
 * (long occlusion, never acquired this session). Both reuse existing, already-hardened
 * machinery; no new pose-matching logic was written for this feature.
 */
bool
Camera::trySeededRecovery(std::unique_ptr<Device> &device,
                          CameraSample &sample,
                          DeviceState &device_state,
                          xrt_pose &Tcv_cam_world,
                          std::optional<xrt_pose> &Tcv_world_device_prior,
                          std::optional<xrt_pose> &Tcv_world_device_last_known,
                          const pose_metrics_trusted_orientation *trusted_orientation)
{
	if (!debug_get_bool_option_constellation_seed_prior()) {
		return false;
	}

	// Reuses WMR_CONSTELLATION_YAW_PRIOR_DEG's own enablement: trusted_orientation is only
	// non-null when getTrustedOrientation (called once per device per sample by our caller)
	// already found a locked, trusted heading -- no separate lock/threshold logic here.
	if (trusted_orientation == nullptr) {
		return false;
	}

	auto tracker = this->tracker;

	// Position source for the seed. REVERSED from the original design (first hardware run,
	// 2026-08-19, everyday-system rig): the predicted prior is preferred over last-known now.
	// The original argument for last-known-first ("a half-turn ghost's position error is
	// small") missed that Device::locked_data.last_known_pose is fed by pushPose's
	// deliberately-UNCONDITIONAL update (see its load-bearing comment) -- it accepts
	// position-garbage poses that never passed any gate, and the seeded attempts themselves
	// then re-feed it, a positive feedback loop measured live: seed positions ran away
	// 4m -> 12m from the camera inside one session while the real controllers sat 1m away.
	// The predicted prior comes from the tracking source's relation history, which only
	// ever stores driver-gate-ACCEPTED samples -- a far cleaner seed. Last-known stays as
	// the fallback for a device that has never had an accepted sample this session.
	const xrt_pose *position_source = nullptr;
	bool seed_from_prior = false;
	if (Tcv_world_device_prior.has_value()) {
		position_source = &Tcv_world_device_prior.value();
		seed_from_prior = true;
	} else if (Tcv_world_device_last_known.has_value()) {
		position_source = &Tcv_world_device_last_known.value();
	} else {
		// Nothing to seed a position with -- a trusted heading alone can't localize the
		// device anywhere in the frame.
		return false;
	}

	// Plausibility clamp (same first-hardware-run finding as above): refuse to seed from a
	// position implausibly far from this camera, so one poisoned entry cannot keep the
	// runaway loop alive. 3m is generous for every source that can currently reach this
	// code -- seeding is gated on get_trusted_orientation, which only the WMR driver
	// implements today, and a WMR controller is handheld near the head-mounted cameras.
	// Revisit the constant if an external-camera rig (rift-style) ever opts in.
	{
		const float SEED_MAX_CAM_DISTANCE_M = 3.0f;
		xrt_pose seed_in_cam;
		math_pose_transform(&Tcv_cam_world, position_source, &seed_in_cam);
		float d2 = seed_in_cam.position.x * seed_in_cam.position.x +
		           seed_in_cam.position.y * seed_in_cam.position.y +
		           seed_in_cam.position.z * seed_in_cam.position.z;
		if (d2 > SEED_MAX_CAM_DISTANCE_M * SEED_MAX_CAM_DISTANCE_M) {
			uint64_t *skip_count = seed_recovery_counter_slot(seed_recovery_skip_count, device->id);
			(*skip_count)++;
			if (*skip_count == 1 || (*skip_count % 500) == 0) {
				CT_INFO(tracker,
				        "constellation seed-prior: SKIPPING seed for device %d -- %s position "
				        "%.2f m from camera exceeds the %.1f m plausibility bound (%llu skips "
				        "so far for this device)",
				        device->id, seed_from_prior ? "prior" : "last-known", sqrtf(d2),
				        SEED_MAX_CAM_DISTANCE_M, (unsigned long long)*skip_count);
			}
			return false;
		}
	}

	float yaw_threshold_rad = 0.0f;
	std::optional<xrt_pose> Tcv_world_trusted_pose =
	    this->getTrustedWorldPose(device, sample.timestamp_ns, &yaw_threshold_rad);
	if (!Tcv_world_trusted_pose.has_value()) {
		// Shouldn't normally happen given trusted_orientation != nullptr above (same
		// callback, same gating) -- stay defensive against a tracking source answering
		// the two queries inconsistently a few lines apart under real concurrency.
		return false;
	}

	xrt_pose Tcv_world_device_seed = *position_source;
	Tcv_world_device_seed.orientation = Tcv_world_trusted_pose->orientation;

	uint64_t *attempt_count = seed_recovery_counter_slot(seed_recovery_attempt_count, device->id);
	(*attempt_count)++;
	if (*attempt_count == 1 || (*attempt_count % 100) == 0) {
		CT_INFO(tracker,
		        "constellation seed-prior: attempting seeded recovery for device %d (seed pos "
		        "%.3f,%.3f,%.3f from %s, %llu attempts so far for this device)",
		        device->id, Tcv_world_device_seed.position.x, Tcv_world_device_seed.position.y,
		        Tcv_world_device_seed.position.z,
		        seed_from_prior ? "predicted prior" : "last-known pose",
		        (unsigned long long)*attempt_count);
	}

	auto log_success = [&](const char *path_desc) {
		uint64_t *success_count = seed_recovery_counter_slot(seed_recovery_success_count, device->id);
		(*success_count)++;
		if (*success_count == 1 || (*success_count % 100) == 0) {
			CT_INFO(tracker,
			        "constellation seed-prior: RECOVERED device %d via seeded %s hypothesis "
			        "(%llu successes / %llu attempts so far for this device)",
			        device->id, path_desc, (unsigned long long)*success_count,
			        (unsigned long long)*attempt_count);
		}
	};

	// Attempt 1: RANSAC-PnP refinement seeded with our (trusted-orientation,
	// chosen-position) hypothesis, using whatever blobs continuity has already labeled
	// for this device (same >=4-LED precondition tryDeviceBlobRecovery's own
	// ransac_pnp_pose call already enforces internally, so this is a cheap no-op when
	// there aren't any).
	if (this->tryDeviceBlobRecovery(device, sample, device_state, Tcv_cam_world, Tcv_world_device_prior,
	                                &Tcv_world_device_seed, trusted_orientation)) {
		log_success("PnP-refined");
		return true;
	}

	// Attempt 2: pure reprojection-based match directly from the seed pose, no PnP
	// refinement -- covers a device with nothing labeled for it at all this frame.
	// Reuses tryDevicePose verbatim, just with our seeded candidate in place of the
	// ordinary predicted/last-known pose it's normally called with.
	if (this->tryDevicePose(device, sample, device_state, Tcv_cam_world, Tcv_world_device_prior,
	                        Tcv_world_device_seed, trusted_orientation)) {
		log_success("reprojection-only");
		return true;
	}

	return false;
}

void
Camera::processSampleSlow(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	auto &data = this->slow_processing_thread_data;

	CT_TRACE(tracker, "Starting slow processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	// Blob-count / swamping guard (docs/40 "Refined fix direction", option 1;
	// WMR_CONSTELLATION_MAX_BLOBS, default 0 = off). The exhaustive correspondence search
	// below is combinatorial in blob count (see correspondence_search.c's file comment), so
	// a frame with an unusually high blob count both costs the most CPU and is the least
	// likely to yield a reliable match -- exactly the pathological case
	// WMR_CONSTELLATION_SEARCH_BUDGET_US (0051) was found NOT to be safe to default on for
	// (it cuts real matches too). Skipping the search entirely for a swamped frame avoids
	// that tradeoff: acquisition just tries again next frame.
	//
	// IMPORTANT, do not set N to "number of LEDs on a controller": the lab rig measured
	// num_blobs=29 with BOTH controllers legitimately in view (2x16 LEDs) -- a threshold
	// anywhere near ~16 would reject real dual-controller frames. There is no universal
	// safe default: the everyday system's own *swamped* baseline (controllers OFF, room
	// light only, i.e. zero real LEDs) measured 22-27 blobs, which overlaps the legitimate
	// dual-controller count above. This guard therefore defaults OFF; enable it with a
	// value measured on the box it runs on, above the highest legitimate blob count that
	// box ever produces.
	long max_blobs = debug_get_num_option_constellation_max_blobs();
	if (max_blobs > 0 && (long)sample.blob_count > max_blobs) {
		auto &swamp_log = data.swamped_blobs_log;
		swamp_log.skipped_since_log++;
		int64_t now_ns = os_monotonic_get_ns();
		if (now_ns >= swamp_log.next_log_ns) {
			CT_INFO(tracker,
			        "Camera %p: skipped %" PRIu64
			        " swamped frame(s) in the last ~5s (blob count over WMR_CONSTELLATION_MAX_BLOBS=%ld, "
			        "latest frame had %u)",
			        (void *)this, swamp_log.skipped_since_log, max_blobs, sample.blob_count);
			swamp_log.skipped_since_log = 0;
			swamp_log.next_log_ns = now_ns + 5 * (int64_t)U_TIME_1S_IN_NS;
		}
		return;
	}

	correspondence_search_set_blobs(data.cs, sample.blobs, sample.blob_count);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	std::shared_lock lock(tracker->device_lock);

	auto Txr_world_cam = sample.Txr_world_cam;

	// OpenCV-convention camera pose in world, computed once per sample (not per device/pass) --
	// used both by the CS_FLAG_MATCH_GRAVITY block below (which used to compute its own copy
	// inline) and by getTrustedOrientation's frame conversion, which needs it regardless of
	// whether a gravity-matching pose prior happens to be available this frame.
	std::optional<xrt_pose> Tcv_world_cam = std::nullopt;
	if (Txr_world_cam.has_value()) {
		xrt_pose tmp;
		math_pose_convert_opencv(&Txr_world_cam.value(), &tmp);
		Tcv_world_cam = tmp;
	}

	for (int i = 0; i < 2; i++) {
		for (std::unique_ptr<Device> &device : tracker->devices) {
			auto search_model = device->search_model;

			// Do a shallow search first go around
			correspondence_search_flags search_flags =
			    i == 0 ? CS_FLAG_SHALLOW_SEARCH : CS_FLAG_DEEP_SEARCH;

			search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_STOP_FOR_STRONG_MATCH);

			auto device_state = sample.getDeviceState(device->id).value_or(nullptr);
			// If there was no device state in the sample, that means this device appeared after the
			// constellation tracker started this sample, so we need to fill out the device state here.
			if (device_state == nullptr) {
				device_state = &sample.putDeviceState(device->id);

				// we need to do a slow process for this device since it wasn't present in the fast
				// processing
				device_state->needs_slow_processing = true;
			}

			if (!device_state->needs_slow_processing) {
				// The device is tracking fine through the fast path; forget any
				// backoff so the next genuine loss starts with fast retries.
				data.deep_backoff.erase(device->id);
				data.lost_search_decimation.erase(device->id);
				continue; // we already did a fast process for this device and it succeeded, no need to
				          // do a slow one
			}

			// Lost-controller search decimation (WMR_CONSTELLATION_LOST_SEARCH_DIV, default 0 =
			// off; docs/40's "controller-present gate" idea, made safe). T197 (live gdb stacks
			// taken during a sustained no-match session) caught pose_metrics' own
			// project_led_points/find_best_matching_led pegging cores -- the cost lives in the
			// model search itself, both shallow and deep passes, not only in the deep-search
			// escalation the backoff below already throttles. The shallow pass runs on every
			// sample "so acquisition is attempted continuously" (see that comment) -- exactly
			// what needs decimating for a device that's genuinely not there (controllers off,
			// long occlusion), not one mid-reacquisition. So: skip the whole per-device search
			// attempt (both passes) outright, but only for N-1 out of every N eligible frames, so
			// it keeps reacquiring -- just at 1/N the cost. E.g. N=10 against 30 Hz cameras still
			// tries reacquisition at ~3 Hz, plenty to catch a controller entering view.
			// "Recently matched" reuses last_known_pose.timestamp_ns, the same last-successful-
			// pose stamp pushPose() already maintains below (no new per-model state needed): within
			// 1s of a real match, decimation never engages, so a controller that IS being tracked
			// is never throttled -- only one that's been lost for a while. Computed once per sample
			// on the i==0 (shallow) pass and re-read on i==1 (deep) so both passes agree; otherwise
			// a decimated shallow pass could still be followed by a full deep pass the same frame.
			auto &decim = data.lost_search_decimation[device->id];
			if (i == 0) {
				bool recently_matched = false;
				{
					std::unique_lock<os::Mutex> lock(device->data_lock);
					if (device->locked_data.last_known_pose.has_value() &&
					    sample.timestamp_ns > device->locked_data.last_known_pose->timestamp_ns) {
						int64_t age_ns = sample.timestamp_ns -
						                 device->locked_data.last_known_pose->timestamp_ns;
						recently_matched = age_ns < (int64_t)U_TIME_1S_IN_NS;
					}
				}

				long lost_search_div = debug_get_num_option_constellation_lost_search_div();
				if (recently_matched || lost_search_div <= 0) {
					decim.frame_count = 0;
					decim.skip_this_sample = false;
				} else {
					decim.frame_count++;
					decim.skip_this_sample = (decim.frame_count % (uint32_t)lost_search_div) != 0;
				}
			}
			if (decim.skip_this_sample) {
				continue;
			}

			// Deep-search backoff: the full-depth combinatorial search is the most
			// expensive thing this tracker does, and for a device that keeps failing
			// it (cold start, occlusion, dim LEDs) running it to completion on every
			// frame both saturates this thread and starves every other device's
			// search -- measured as ~2 cores across the 4 slow threads with the
			// solution rate collapsed to one fix every ~3 s. The shallow pass still
			// runs on every sample so acquisition is attempted continuously; only
			// the deep escalation is rate-limited, doubling from 50 ms up to a
			// ceiling of 800 ms while it keeps failing.
			auto &backoff = data.deep_backoff[device->id];
			if (i == 1 && backoff.next_attempt_ns != 0 &&
			    os_monotonic_get_ns() < backoff.next_attempt_ns) {
				continue;
			}

			xrt_pose Tcv_cam_device = XRT_POSE_IDENTITY;
			if (device_state->Txr_world_device_prior.has_value() && Txr_world_cam.has_value()) {
				xrt_pose Txr_cam_world;
				math_pose_invert(&Txr_world_cam.value(), &Txr_cam_world);

				xrt_pose Txr_cam_device;
				math_pose_transform(&Txr_cam_world, &device_state->Txr_world_device_prior.value(),
				                    &Txr_cam_device);

				math_pose_convert_opencv(&Txr_cam_device, &Tcv_cam_device);

				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_HAVE_POSE_PRIOR);
			}

			// Arbitrary threshold to prevent trusting a gravity vector if the device itself isn't confident
			// in it's own gravity.
			const float gravity_error_threshold_deg = 25.f;

			xrt_vec3 cv_camera_gravity_vector = {0.0, 1.0, 0.0};
			if ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0 &&
			    device->gravity_error_rad < DEG_TO_RAD(gravity_error_threshold_deg)) {
				// If we have a pose for the camera and we have a prior pose
				// (required by correspondence for search gravity matching)
				if (Tcv_world_cam.has_value()) {
					// Acquire the camera's gravity vector under the processing lock
					get_pose_gravity_vector(Tcv_world_cam.value(), cv_camera_gravity_vector);

					// Add in to check gravity
					search_flags =
					    (correspondence_search_flags)(search_flags | CS_FLAG_MATCH_GRAVITY);
				}
			}

			// Trusted-orientation yaw reference (reverb-g2 T213, patch 0074 follow-up), if the
			// device's tracking source has one available -- independent of CS_FLAG_HAVE_POSE_PRIOR/
			// CS_FLAG_MATCH_GRAVITY above (a device can have a locked fusion heading well before it
			// ever has a position prior). See getTrustedOrientation's own doc comment.
			std::optional<pose_metrics_trusted_orientation> trusted_orientation = std::nullopt;
			if (Tcv_world_cam.has_value()) {
				trusted_orientation =
				    this->getTrustedOrientation(device, Tcv_world_cam.value(), sample.timestamp_ns);
			}
			if (trusted_orientation.has_value()) {
				search_flags =
				    (correspondence_search_flags)(search_flags | CS_FLAG_HAVE_TRUSTED_ORIENTATION);
			}
			const pose_metrics_trusted_orientation *trusted_orientation_ptr =
			    trusted_orientation.has_value() ? &trusted_orientation.value() : nullptr;

			pose_metrics score;
			bool found_pose = correspondence_search_find_one_pose( //
			    data.cs,                                           //
			    search_model,                                      //
			    search_flags,                                      //
			    &Tcv_cam_device,                                   //
			    &device->prior_pos_error,                          //
			    &device->prior_rot_error,                          //
			    &cv_camera_gravity_vector,                         //
			    device->gravity_error_rad,                         //
			    &score,                                            //
			    trusted_orientation_ptr);                          //
			if (found_pose) {
				this->pushPose(sample,               //
				               *device_state,        //
				               device,               //
				               score,                //
				               Tcv_cam_device,       //
				               false,                //
				               trusted_orientation_ptr); //

				// We found a pose for this device in this sample
				device_state->needs_slow_processing = false;
				data.deep_backoff.erase(device->id);
				data.lost_search_decimation.erase(device->id);
			} else {
				CT_TRACE(tracker, "Camera %p slow processing for device %d failed to find a pose",
				         (void *)this, device->id);
				if (i == 1) {
					backoff.consecutive_failures++;
					int shift = backoff.consecutive_failures - 1;
					if (shift > 4) {
						shift = 4; // 50ms << 4 = 800ms ceiling
					}
					backoff.next_attempt_ns =
					    os_monotonic_get_ns() + (int64_t)(50 * U_TIME_1MS_IN_NS << shift);
				}
			}
		}
	}

	this->debugScribbleSample(sample, false);

#ifdef XRT_FEATURE_RERUN
	// If a slow sample was triggered by the fast processing thread, we always want to log the sample.
	if (tracker->rerun_stream) {
		tracker->rerun_stream->logSample(*tracker, sample);
	}
#endif
}

bool
Camera::processSampleFast(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	CT_TRACE(tracker, "Starting fast processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	auto Txr_world_cam = sample.Txr_world_cam;
	if (!Txr_world_cam.has_value()) {
		CT_TRACE(tracker, "Camera %p has no world pose, cannot do fast processing", (void *)this);
		return false;
	}

	xrt_pose Tcv_world_cam;
	math_pose_convert_opencv(&Txr_world_cam.value(), &Tcv_world_cam);

	xrt_pose Tcv_cam_world;
	math_pose_invert(&Tcv_world_cam, &Tcv_cam_world);

	bool need_slow_search = false;
	std::shared_lock lock(tracker->device_lock);
	for (std::unique_ptr<Device> &device : tracker->devices) {
		xrt_space_relation device_predicted_relation = XRT_SPACE_RELATION_ZERO; //< AKA "the prior"

		if (device->params.tracking_source != nullptr) {
			t_constellation_tracker_tracking_source_get_tracked_pose(
			    device->params.tracking_source, sample.timestamp_ns, &device_predicted_relation);
		}

		std::optional<xrt_pose> Tcv_world_device_predicted = std::nullopt; //< AKA "the prior"
		// Whether the prior pose is actually valid
		if ((device_predicted_relation.relation_flags &
		     (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) ==
		    (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) {
			Tcv_world_device_predicted.emplace(xrt_pose{}); // initialize it to a value
			math_pose_convert_opencv(&device_predicted_relation.pose, &Tcv_world_device_predicted.value());
		}

		auto &device_state = sample.putDeviceState(device->id);
		device_state.Txr_world_device_prior = Tcv_world_device_predicted.has_value()
		                                          ? std::optional<xrt_pose>(device_predicted_relation.pose)
		                                          : std::nullopt;

		// Trusted-orientation yaw reference (reverb-g2 T213, patch 0074 follow-up), if the device's
		// tracking source has one available -- see getTrustedOrientation's own doc comment and
		// processSampleSlow's identical use of it. Computed once per device per sample, independent
		// of Tcv_world_device_predicted above (a locked fusion heading can exist well before any
		// position prior does).
		auto trusted_orientation = this->getTrustedOrientation(device, Tcv_world_cam, sample.timestamp_ns);
		const pose_metrics_trusted_orientation *trusted_orientation_ptr =
		    trusted_orientation.has_value() ? &trusted_orientation.value() : nullptr;

		bool wipe_blob_associations = false;
		if (this->tryDeviceBlobRecovery(device, sample, device_state, Tcv_cam_world,
		                                Tcv_world_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with blob recovery", device->id);
			continue; // try the next device, we found a pose!
		} else {
			wipe_blob_associations = true;
		}

		// if we have a valid prior pose, try to use it for fast matching
		if (Tcv_world_device_predicted.has_value() &&
		    this->tryDevicePose(device, sample, device_state, Tcv_cam_world, Tcv_world_device_predicted,
		                        Tcv_world_device_predicted.value(), trusted_orientation_ptr)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded", device->id);
			continue; // try the next device, we found a pose!
		}

		// Try to get a last known pose
		bool has_last_known = false;
		xrt_pose Tcv_world_device_last_known;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);

			if (auto last_known_pose = device->locked_data.last_known_pose) {
				math_pose_convert_opencv(&last_known_pose->Txr_world_device,
				                         &Tcv_world_device_last_known);
				has_last_known = true;
			}
		}

		if (has_last_known &&
		    this->tryDevicePose(device, sample, device_state, Tcv_cam_world, Tcv_world_device_predicted,
		                        Tcv_world_device_last_known, trusted_orientation_ptr)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with last known pose", device->id);
			continue; // try the next device, we found a pose!
		}

		// Assignment-prior SEEDING (reverb-g2 T215, WMR_CONSTELLATION_SEED_PRIOR, default
		// off -- see Camera::trySeededRecovery's own doc comment for the full design). Last
		// fast-path attempt: every ordinary fast path above already failed this frame, so
		// this device is about to either lose its blob associations or fall through to the
		// far less frequent slow search -- try one more, cheap, trusted-heading-seeded
		// hypothesis first. Reuses has_last_known/Tcv_world_device_last_known computed just
		// above instead of re-querying device->data_lock a second time.
		std::optional<xrt_pose> Tcv_world_device_last_known_opt =
		    has_last_known ? std::optional<xrt_pose>(Tcv_world_device_last_known) : std::nullopt;
		if (this->trySeededRecovery(device, sample, device_state, Tcv_cam_world, Tcv_world_device_predicted,
		                            Tcv_world_device_last_known_opt, trusted_orientation_ptr)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with seeded recovery", device->id);
			continue; // try the next device, we found a pose!
		}

		if (wipe_blob_associations) {
			// Blob-based recovery failed, unmark all blobs for this device.
			for (uint32_t i = 0; i < sample.blob_count; i++) {
				t_blob &b = sample.blobs[i];
				if (b.matched_device_id == device->id) {
					b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
					b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				}
			}
		}

		device_state.needs_slow_processing = true;

		need_slow_search = true;
	}

	this->debugScribbleSample(sample, true);

	// Only save samples on the fast processing thread, since the slow processing thread is *triggered* by the fast
	// processing thread.
	if (tracker->data_recorder) {
		tracker->data_recorder->recordSample(sample);
	}

#ifdef XRT_FEATURE_RERUN
	// We only want to log this sample to rerun if we aren't about to do a full search
	if (!need_slow_search && tracker->rerun_stream) {
		tracker->rerun_stream->logSample(*tracker, sample);
	}
#endif

	return need_slow_search;
}

void
Camera::pushPose(CameraSample &camera_sample,
                 DeviceState &device_state,
                 std::unique_ptr<Device> &device,
                 pose_metrics &score,
                 xrt_pose &Tcv_cam_device,
                 bool was_optimized,
                 const pose_metrics_trusted_orientation *trusted_orientation)
{
	// We should never find two poses for the same device in a single frame
	assert(device_state.found_pose.has_value() == false);

	ConstellationTracker *tracker = this->tracker;

	uint32_t blobs_marked_before_update = num_blobs_for_device(camera_sample, device->id);

	// Match visible blobs to the pose we found
	pose_metrics_blob_match_info blob_match_info;
	pose_metrics_match_pose_to_blobs(&Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
	                                 &device->params.led_model, device->id, &this->model, &blob_match_info);

	// Mark all the new blobs using the match info
	camera_sample.markMatchingBlobs(tracker, device->params.led_model, device->id, blob_match_info);

	// Only do an optimization if we haven't already optimized, or we marked new blobs.
	// This prevents us from optimizing a pose multiple times in a single frame.
	if (!was_optimized || num_blobs_for_device(camera_sample, device->id) > blobs_marked_before_update) {
		// Try to optimize the pose again, unmarking outliers
		uint32_t num_leds_out;
		uint32_t num_inliers;
		if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
		                     &device->params.led_model, device->id, &this->model, &num_leds_out,
		                     &num_inliers)) {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs failed",
			         0, 0, device->id, camera_sample.blob_count);
		} else {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs had %d LEDs with %d inliers. Produced pose %f,%f,%f,%f pos %f,%f,%f",
			         0, 0, device->id, camera_sample.blob_count, num_leds_out, num_inliers,
			         Tcv_cam_device.orientation.x, Tcv_cam_device.orientation.y,
			         Tcv_cam_device.orientation.z, Tcv_cam_device.orientation.w, Tcv_cam_device.position.x,
			         Tcv_cam_device.position.y, Tcv_cam_device.position.z);
		}

		// We need to re-evaluate the pose after optimization, since the reprojection error may have changed.
		// Re-checked against trusted_orientation too: RANSAC-PnP can move the pose, so a candidate that
		// agreed with the trusted heading before refinement is not guaranteed to still agree after it.
		pose_metrics_evaluate_pose(&score, &Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
		                           &device->params.led_model, device->id, &this->model, NULL,
		                           trusted_orientation);
	}

	// Move to OpenXR space
	xrt_pose Txr_cam_device;
	math_pose_convert_opencv(&Tcv_cam_device, &Txr_cam_device);

	CT_DEBUG(tracker, "Pose: orient %f %f %f %f pos %f %f %f", Txr_cam_device.orientation.x,
	         Txr_cam_device.orientation.y, Txr_cam_device.orientation.z, Txr_cam_device.orientation.w,
	         Txr_cam_device.position.x, Txr_cam_device.position.y, Txr_cam_device.position.z);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	float average_brightness = 0.0f;
	uint32_t used_blobs = 0;
	for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
		struct t_blob &b = camera_sample.blobs[i];
		if (b.matched_device_id == device->id) {
			average_brightness += b.brightness;
			used_blobs++;
		}
	}

	if (used_blobs > 0) {
		average_brightness /= used_blobs;
	} else {
		average_brightness = 1.0f;
	}

	// Mark that we found a pose
	device_state.found_pose = {
	    .Tcv_cam_device = Tcv_cam_device,
	    .average_blob_brightness = average_brightness,
	};

	auto Txr_world_cam = camera_sample.Txr_world_cam;
	if (!Txr_world_cam.has_value()) {
		// Can't do anything if we can't locate the camera in the world.
		return;
	}

	xrt_pose Txr_world_device;
	math_pose_transform(&Txr_world_cam.value(), &Txr_cam_device, &Txr_world_device);

	// Compute the metrics
	t_constellation_tracker_sample_metrics metrics = {
	    .matched_blob_count = score.matched_blobs,
	    .visible_led_count = score.visible_leds,
	    .reprojection_error = score.matched_blobs > 0 ? sqrtf(score.reprojection_error / score.matched_blobs) : 0.0,
	};

	// Both callers gate on POSE_MATCH_GOOD before calling this function, but that is the score of
	// the pose BEFORE the RANSAC-PnP refinement above, which moves the pose and can drop LEDs.
	// The refined score is recomputed right after it and, until this check, was only ever used to
	// fill in the metrics below -- so a pose the tracker's own criterion no longer considers good
	// was still published to the driver.
	//
	// Measured on an HP Reverb G2, 876 samples, 2026-08-12: samples arriving at the driver with
	// matched_blob_count == 0 (a pose supported by no observation at all) and with 2 blobs at
	// 2.24 px reprojection error. Each one lands as a jump in the controller's position -- the
	// worst was 1.6 m, and the wearer feels them as the hand snapping to an unexpected side.
	// Note the giveaway that a plain reprojection-error threshold would NOT have caught: several
	// of the bad solves have a very LOW error (0.11 px), because with few visible LEDs a wrong
	// pose can fit the handful of blobs almost perfectly. The match flags account for that; a
	// hand-picked error threshold does not.
	//
	// Only the publication is suppressed: found_pose, the marked blobs and the rest of the
	// tracker's per-frame bookkeeping are left exactly as they were, so this changes what the
	// device is told and nothing about how the tracker searches.
	//
	// trusted_orientation (reverb-g2 T213, patch 0074 follow-up) rides this SAME gate for free:
	// pose_metrics_evaluate_pose above was given trusted_orientation, so a refined pose that
	// disagrees with it on yaw already comes back without POSE_MATCH_GOOD (see
	// pose_metrics_evaluate_pose_with_prior's loophole-branch check) -- no separate check needed
	// here. What trusted_orientation ALSO does, unlike an ordinary failed prior match, is gate
	// the locked_data update just below -- see that block's own comment.
	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
		// Push the sample to the device
		t_constellation_tracker_sample sample = {
		    .timestamp_ns = camera_sample.timestamp_ns,
		    .pose = Txr_world_device,
		    .mosaic_index = mosaic->index,
		    .camera_index = this->index,
		    .average_brightness = average_brightness, // @todo compute this
		    .metrics = metrics,
		};
		t_constellation_tracker_device_push_sample(device->device, &sample);
	} else {
		CT_DEBUG(tracker,
		         "Device %d: pose no longer POSE_MATCH_GOOD after refinement (%u matched blobs, %u "
		         "visible LEDs) -- not publishing it",
		         device->id, score.matched_blobs, score.visible_leds);
	}

	{
		std::unique_lock<os::Mutex> lock(device->data_lock);

		// last_known_pose/blob-marking below are normally updated UNCONDITIONALLY, even for a pose
		// that didn't pass POSE_MATCH_GOOD after refinement (see the publication gate above, added
		// 2026-08-12): only the publication to the device is suppressed, deliberately, so a
		// so-so pose still feeds the tracker's own internal recovery prior. Do NOT relax that for
		// the general case -- it is load-bearing for ordinary reacquisition.
		//
		// trusted_orientation (reverb-g2 T213) is the one narrow exception, and only when it was
		// actually supplied for this call: a candidate that disagrees with a LOCKED, always-fresh
		// fusion heading on yaw is not "so-so", it's the yaw-ghost this whole feature exists to
		// keep out of the tracker's own state -- 0074's own follow-up note named this exact update
		// as the residual way a rejected ghost still poisons future searches. When
		// trusted_orientation is nullptr (every device/camera that hasn't opted in, i.e. almost
		// all of them), trusted_yaw_ok is unconditionally true and this block is BYTE IDENTICAL to
		// before.
		bool trusted_yaw_ok = trusted_orientation == nullptr || POSE_HAS_FLAGS(&score, POSE_MATCH_TRUSTED_YAW);

		if (trusted_yaw_ok) {
			// If we already found a pose in the future, then don't mark blobs, since the device has
			// definitely moved.
			if (!device->locked_data.last_known_pose.has_value() ||
			    device->locked_data.last_known_pose->timestamp_ns <= camera_sample.timestamp_ns) {
				// Call back to the blobwatch to update the blobs for this device. Done after pose
				// optimization since the RANSAC process will unlabel any outliers.
				auto tbo = camera_sample.toBlobObservation();
				t_blobwatch_mark_blob_device(camera_sample.source, &tbo, device->id);
			}

			device->locked_data.last_known_pose = DeviceLastPose(Txr_world_device, camera_sample.timestamp_ns);
		} else {
			CT_DEBUG(tracker,
			         "Device %d: pose disagreed with trusted orientation on yaw (%.1f deg) -- not letting "
			         "it poison last_known_pose/blob associations",
			         device->id, RAD_TO_DEG(score.trusted_yaw_error_rad));
		}
	}

	CT_DEBUG(tracker, "Found pose for device %d", device->id);
}

std::optional<xrt_pose>
Camera::getTrustedWorldPose(std::unique_ptr<Device> &device, int64_t when_ns, float *out_yaw_threshold_rad)
{
	if (device->params.tracking_source == nullptr) {
		return std::nullopt;
	}

	// Contract (see t_constellation.h): out_orientation comes back in the SAME xrt world/
	// tracking-origin frame and body-frame convention get_tracked_pose's own poses use --
	// converting from whatever internal reference the tracking source has (e.g. a WMR
	// controller's IMU fusion) into that convention is the tracking source's own job, not ours.
	xrt_quat Txr_world_trusted_orientation;
	float yaw_threshold_rad = 0.0f;
	if (!t_constellation_tracker_tracking_source_get_trusted_orientation(
	        device->params.tracking_source, when_ns, &Txr_world_trusted_orientation, &yaw_threshold_rad)) {
		return std::nullopt;
	}
	if (!(yaw_threshold_rad > 0.0f)) {
		// A non-positive threshold can't ever be satisfied meaningfully -- treat it the same as
		// "nothing trustworthy right now" rather than rejecting every single candidate.
		return std::nullopt;
	}

	xrt_pose Txr_world_trusted_pose = XRT_POSE_IDENTITY;
	Txr_world_trusted_pose.orientation = Txr_world_trusted_orientation;
	xrt_pose Tcv_world_trusted_pose;
	math_pose_convert_opencv(&Txr_world_trusted_pose, &Tcv_world_trusted_pose);

	if (out_yaw_threshold_rad != nullptr) {
		*out_yaw_threshold_rad = yaw_threshold_rad;
	}
	return Tcv_world_trusted_pose;
}

std::optional<pose_metrics_trusted_orientation>
Camera::getTrustedOrientation(std::unique_ptr<Device> &device, xrt_pose &Tcv_world_cam, int64_t when_ns)
{
	// Convert into THIS camera's frame -- the exact same convert-then-transform-then-invert shape
	// tryDevicePose's own Tcv_cam_device_prior conversion uses a few lines above this function's
	// callers, just applied to an orientation-only pose (position is irrelevant to the result: pose
	// composition's rotation output never depends on the child pose's translation).
	float yaw_threshold_rad = 0.0f;
	std::optional<xrt_pose> Tcv_world_trusted_pose = this->getTrustedWorldPose(device, when_ns, &yaw_threshold_rad);
	if (!Tcv_world_trusted_pose.has_value()) {
		return std::nullopt;
	}

	xrt_pose Tcv_cam_world;
	math_pose_invert(&Tcv_world_cam, &Tcv_cam_world);

	xrt_pose Tcv_cam_trusted_pose;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_trusted_pose.value(), &Tcv_cam_trusted_pose);

	pose_metrics_trusted_orientation trusted{};
	trusted.orientation = Tcv_cam_trusted_pose.orientation;
	// "World up as seen from this camera" -- identical quantity to CS_FLAG_MATCH_GRAVITY's own
	// cv_camera_gravity_vector (processSampleSlow), computed independently here since that one is
	// only ever filled in conditionally (gated on device->gravity_error_rad and a position prior
	// existing), neither of which trusted_orientation depends on.
	get_pose_gravity_vector(Tcv_world_cam, trusted.up_vector);
	trusted.yaw_threshold_rad = yaw_threshold_rad;

	return trusted;
}

/*
 *
 * CameraMosaic implementations
 *
 */

CameraMosaic::CameraMosaic(ConstellationTracker *tracker,
                           const t_constellation_tracker_camera_mosaic &mosaic_params,
                           size_t index)
    : index(index)
{
	this->tracking_origin = mosaic_params.tracking_origin;

	// NOTE: the stability of this vector is important since we're passing to C callbacks and APIs!
	this->cameras.reserve(mosaic_params.num_cameras);
}

std::optional<xrt_pose>
CameraMosaic::getTrackingOriginPose(timepoint_ns when_ns)
{
	if (this->tracking_origin) {
		xrt_space_relation relation;
		t_constellation_tracker_tracking_source_get_tracked_pose(this->tracking_origin, when_ns, &relation);

		// If the tracking source has a valid position, grab it
		if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			return relation.pose;
		} else {
			return std::nullopt;
		}
	}

	return std::optional<xrt_pose>(XRT_POSE_IDENTITY);
}

/*
 *
 * DeviceLastPose implementations
 *
 */

DeviceLastPose::DeviceLastPose(xrt_pose Txr_world_device, timepoint_ns timestamp_ns)
    : Txr_world_device(Txr_world_device), timestamp_ns(timestamp_ns)
{}

/*
 *
 * Device implementations
 *
 */

Device::Device(t_constellation_tracker_device_params *params,
               t_constellation_tracker_device *device,
               t_constellation_device_id_t id)
    : params(*params), device(device), id(id), data_lock(), locked_data({.last_known_pose = std::nullopt})
{
	// Copy the LED model leds into safe memory, since we want to mutate it into OpenCV space
	this->params.led_model.leds = new t_constellation_tracker_led[this->params.led_model.led_count];
	memcpy(this->params.led_model.leds, params->led_model.leds,
	       sizeof(t_constellation_tracker_led) * this->params.led_model.led_count);

	// flip all LEDs from OpenXR -> OpenCV coordinate space, since the tracker works in OpenCV space
	for (size_t i = 0; i < this->params.led_model.led_count; i++) {
		t_constellation_tracker_led &dst = this->params.led_model.leds[i];
		t_constellation_tracker_led &src = params->led_model.leds[i];

		dst = src;
		dst.position.y = -dst.position.y;
		dst.position.z = -dst.position.z;
		dst.normal.y = -dst.normal.y;
		dst.normal.z = -dst.normal.z;
	}

	this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
}

Device::~Device()
{
	if (this->search_model) {
		t_constellation_search_model_free(this->search_model);
		this->search_model = nullptr;
	}

	if (this->params.led_model.leds) {
		delete[] this->params.led_model.leds;
		this->params.led_model.leds = nullptr;
	}
}

/*
 *
 * ConstellationTracker implementations
 *
 */

ConstellationTracker::ConstellationTracker(t_constellation_tracker_params *params)
{
	// If the deterministic flag is set, we force single-threaded processing.
	if ((params->flags & T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC) != 0) {
		this->single_threaded = true;
	}

	this->log_level = debug_get_log_option_constellation_tracker_log();

	this->mosaics.reserve(params->num_mosaics);

	// Fill in our internal data structures based on the provided params
	for (size_t mosaic_idx = 0; mosaic_idx < params->num_mosaics; mosaic_idx++) {
		const t_constellation_tracker_camera_mosaic &mosaic_params = params->mosaics[mosaic_idx];

		std::shared_ptr<CameraMosaic> mosaic = std::make_shared<CameraMosaic>(this, mosaic_params, mosaic_idx);

		// Assert pointer stability!
		assert(mosaic->cameras.capacity() >= mosaic_params.num_cameras);

		for (size_t cam_idx = 0; cam_idx < mosaic_params.num_cameras; cam_idx++) {
			const t_constellation_tracker_camera &camera_params = mosaic_params.cameras[cam_idx];

			// This can't be in the constructor since the `shared_ptr` of the `mosaic` isn't formed
			// yet, but the camera needs to own a weak_ptr to it's mosaic.
			mosaic->cameras.push_back(
			    std::make_unique<Camera>(this, mosaic, camera_params, &this->log_level, cam_idx));
		}

		this->mosaics.push_back(mosaic);
	}

	// Fill in the blob sinks for each camera
	for (size_t i = 0; i < this->mosaics.size(); i++) {
		std::shared_ptr<CameraMosaic> &mosaic = this->mosaics[i];

		for (size_t j = 0; j < mosaic->cameras.size(); j++) {
			Camera *camera = mosaic->cameras[j].get();

			params->mosaics[i].cameras[j].blob_sink = &camera->base;
		}
	}

	this->params = *params;

	std::string data_recorder_output = debug_get_option_constellation_tracker_data_recorder_output();
	if (!data_recorder_output.empty()) {
		this->data_recorder = std::make_unique<DataRecorder>(this, data_recorder_output);
		CT_INFO(this, "Constellation tracker data recorder enabled, outputting to %s",
		        data_recorder_output.c_str());
	}

	if (debug_get_bool_option_constellation_tracker_enable_rerun()) {
#ifdef XRT_FEATURE_RERUN
		this->rerun_stream = std::make_unique<RerunContext>();
		CT_INFO(this, "Constellation tracker Rerun stream enabled, outputting to constellation_tracker.rerun");

		if (debug_get_bool_option_constellation_tracker_rerun_spawn()) {
			this->rerun_stream->stream->spawn().exit_on_failure();
		} else {
			this->rerun_stream->stream->connect_grpc().exit_on_failure();
		}
#else
		CT_ERROR(this, "Rerun stream requested but XRT_FEATURE_RERUN is not enabled");
#endif
	}

	CT_DEBUG(this, "Created constellation tracker with %zu mosaics", this->mosaics.size());
}

ConstellationTracker::~ConstellationTracker()
{
	CT_DEBUG(this, "Destroying constellation tracker");
}

void
ConstellationTracker::setupVariableTracking()
{
	u_var_add_root(this, "Constellation Tracker", true);
	u_var_add_log_level(this, &this->log_level, "Log Level");

	uint32_t mosaic_idx = 0;
	for (auto &mosaic : this->mosaics) {
		uint32_t camera_idx = 0;
		for (auto &camera : mosaic->cameras) {
			auto str =
			    "Camera " + std::to_string(camera_idx) + " (Mosaic " + std::to_string(mosaic_idx) + ")";
			u_var_add_gui_header(this, nullptr, str.c_str());

			u_var_add_sink_debug(this, &camera->fast_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Fast)");
			u_var_add_sink_debug(this, &camera->slow_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Slow)");

			u_var_add_pose(this, &camera->locked_data.Txr_origin_cam, "Camera Origin Pose");
			u_var_add_bool(this, &camera->locked_data.has_concrete_pose, "Has Concrete Pose");

			camera->scribble_settings.setupDebugTracking(this);

			camera_idx++;
		}
		mosaic_idx++;
	}
}

t_constellation_device_id_t
ConstellationTracker::addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device)
{
	std::unique_lock lock(this->device_lock);

	if (this->devices.size() >= XRT_CONSTELLATION_MAX_DEVICES) {
		throw std::runtime_error("Maximum number of devices already added to constellation tracker");
	}

	t_constellation_device_id_t id = this->next_device_id++;

	this->devices.push_back(std::make_unique<Device>(params, device, id));

	CT_DEBUG(this, "Added device with ID %d to constellation tracker", id);

	if (this->data_recorder) {
		this->data_recorder->recordDeviceInfo(*this->devices.back());
	}

	return id;
}

void
ConstellationTracker::removeDevice(t_constellation_device_id_t device_id)
{
	std::unique_lock lock(this->device_lock);

	size_t index = 0;
	for (auto &device : this->devices) {
		if (device->id == device_id) {
			break;
		}
		index++;
	}

	if (index == this->devices.size()) {
		throw std::invalid_argument("The device ID is not present in the device list.");
	}

	// Remove the device
	this->devices.erase(this->devices.begin() + index);
}

}; // namespace xrt::tracking::constellation

using namespace xrt::tracking::constellation;

void *
constellation_tracker_camera_slow_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	// T204 round-2 item: this is the num_blobs=29-class correspondence-search work T197
	// found competing with the compositor for CPU. It runs at default SCHED_OTHER (no RT
	// change here -- it is bulk compute, not a deadline), but can be kept off whichever
	// cores XRT_COMPOSITOR_CPU_AFFINITY reserves for the compositor by setting
	// WMR_CPU_AFFINITY to the complement set. No-op unless set.
	u_try_to_set_thread_affinity_from_env(U_LOGGING_INFO, "Constellation: Slow", "WMR_CPU_AFFINITY");

	os_thread_helper_lock(&camera->slow_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->slow_processing_thread)) {
		os_thread_helper_wait_locked(&camera->slow_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->slow_processing_thread_data.sample;
		camera->slow_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->slow_processing_thread);

		if (auto sample = maybe_sample) {
			camera->processSampleSlow(*sample);
		}

		os_thread_helper_lock(&camera->slow_processing_thread);
	}
	os_thread_helper_unlock(&camera->slow_processing_thread);

	return NULL;
}

void *
constellation_tracker_camera_fast_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	// See the matching call in the slow thread above -- same WMR_CPU_AFFINITY, no-op
	// unless set.
	u_try_to_set_thread_affinity_from_env(U_LOGGING_INFO, "Constellation: Fast", "WMR_CPU_AFFINITY");

	os_thread_helper_lock(&camera->fast_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->fast_processing_thread)) {
		os_thread_helper_wait_locked(&camera->fast_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->fast_processing_thread_data.sample;
		camera->fast_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->fast_processing_thread);

		if (auto sample = maybe_sample) {
			if (camera->processSampleFast(*sample)) {
				CT_TRACE(camera->tracker,
				         "Fast processing for camera %p failed, deferring to slow thread",
				         (void *)camera);
				camera->deferSampleToSlowThread(*sample);
			}
		}

		os_thread_helper_lock(&camera->fast_processing_thread);
	}
	os_thread_helper_unlock(&camera->fast_processing_thread);

	return NULL;
}

void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo)
{
	Camera *camera = Camera::Get(tbs);
	ConstellationTracker *tracker = camera->tracker;

	CT_TRACE(tracker, "Received blob observation with %u blobs", tbo->num_blobs);

	if (tbo->num_blobs == 0) {
		CT_TRACE(tracker, "No blobs in observation, skipping processing");
		return;
	}

	if (tracker->single_threaded) {
		auto sample = CameraSample(*tbo, camera);

		// If we're in single-threaded mode, just process the sample immediately on the fast thread
		if (camera->processSampleFast(sample)) {
			CT_TRACE(tracker,
			         "Fast processing for camera %p failed in single-threaded mode, doing slow processing",
			         (void *)camera);

			camera->processSampleSlow(sample);
		}
	} else {
		// Send to the fast thread
		os_thread_helper_lock(&camera->fast_processing_thread);
		camera->fast_processing_thread_data.sample = CameraSample(*tbo, camera);
		os_thread_helper_signal_locked(&camera->fast_processing_thread);
		os_thread_helper_unlock(&camera->fast_processing_thread);
	}
}

void
constellation_tracker_camera_destroy(t_blob_sink *tbs)
{
	// do nothing, the constellation tracker will clean up the blob sinks when it is destroyed.
}

void
constellation_tracker_node_break_apart(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->running = false;

	// Stop all the threads
	for (auto &mosaic : tracker->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->slow_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->slow_processing_thread);
			}

			if (camera->fast_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->fast_processing_thread);
			}
		}
	}
}

void
constellation_tracker_node_destroy(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	u_var_remove_root(tracker);

	delete tracker;
}

int
t_constellation_tracker_create(xrt_frame_context *xfctx,
                               t_constellation_tracker_params *params,
                               t_constellation_tracker **out_tracker)
{
	try {
		ConstellationTracker *tracker = new ConstellationTracker(params);

		// Add us to the frame context
		xrt_frame_context_add(xfctx, &tracker->node);

		*out_tracker = (t_constellation_tracker *)tracker;

		tracker->setupVariableTracking();
	} catch (const std::exception &e) {
		U_LOG_E("Failed to create constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_add_device(t_constellation_tracker *raw_tracker,
                                   t_constellation_tracker_device_params *params,
                                   t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		t_constellation_device_id_t device_id = tracker->addDevice(params, device);
		*out_device_id = device_id;
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_remove_device(t_constellation_tracker *raw_tracker, t_constellation_device_id_t device)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->removeDevice(device);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to remove device from constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

xrt_tracking_origin *
t_constellation_tracker_get_tracking_origin(t_constellation_tracker *raw_tracker)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	return &tracker->tracking_origin;
}
