// Copyright 2020-2023 Jan Schmidt
// Copyright 2025-2026 Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Metrics for constellation tracking poses
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_constellation.h"

#include "camera_model.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MAX_OBJECT_LEDS 64

#define WORST_REPROJECTION_ERROR 10.0

struct pose_rect
{
	double left;
	double top;
	double right;
	double bottom;
};

XRT_MAYBE_UNUSED static bool
pose_rect_has_area(struct pose_rect *rect)
{
	return rect->left != rect->right && rect->top != rect->bottom;
}

enum pose_match_flags
{
	//! A reasonable pose match - most LEDs matched to within a few pixels error
	POSE_MATCH_GOOD = 0x1,
	//! A strong pose match is a match with very low error
	POSE_MATCH_STRONG = 0x2,
	//! The position of the pose matched the prior well
	POSE_MATCH_POSITION = 0x10,
	//! The orientation of the pose matched the prior well
	POSE_MATCH_ORIENT = 0x20,
	//! If a pose prior was supplied when calculating the score, then rot/trans_error are set
	POSE_HAD_PRIOR = 0x100,
	//! The LED IDs on the blobs all matched the LEDs we thought (or were unassigned)
	POSE_MATCH_LED_IDS = 0x200,
	//! A @ref pose_metrics_trusted_orientation was supplied for this evaluation (i.e.
	//! pose_metrics_evaluate_pose[_with_prior]'s trusted_orientation param was non-NULL) --
	//! lets callers (e.g. pose_metrics_score_is_better_pose) tell "the check ran and this pose
	//! failed it" (this flag set, POSE_MATCH_TRUSTED_YAW clear) apart from "no trusted
	//! orientation was available for this comparison at all" (this flag clear, in which case
	//! POSE_MATCH_TRUSTED_YAW is meaningless and must not be consulted).
	POSE_HAD_TRUSTED_ORIENTATION = 0x400,
	//! Only meaningful together with POSE_HAD_TRUSTED_ORIENTATION: the pose's yaw (about the
	//! trusted orientation's up_vector) agreed with it within yaw_threshold_rad.
	POSE_MATCH_TRUSTED_YAW = 0x800,
};

#define POSE_SET_FLAG(score, f) ((score)->match_flags |= (f))
#define POSE_CLEAR_FLAG(score, f) ((score)->match_flags &= ~(f))
#define POSE_HAS_FLAGS(score, f) (((score)->match_flags & (f)) == (f))

struct pose_metrics
{
	enum pose_match_flags match_flags;

	uint32_t matched_blobs;
	uint32_t unmatched_blobs;
	uint32_t visible_leds;

	double reprojection_error;

	//! Rotation error (compared to a prior)
	struct xrt_vec3 orient_error;
	//! Translation error (compared to a prior)
	struct xrt_vec3 pos_error;

	//! Signed yaw error (radians) against a supplied trusted_orientation, about its up_vector.
	//! Only meaningful when POSE_HAD_TRUSTED_ORIENTATION is set in match_flags -- otherwise
	//! left uninitialized by pose_metrics_evaluate_pose[_with_prior], same as orient_error/
	//! pos_error above are only meaningful under POSE_HAD_PRIOR.
	double trusted_yaw_error_rad;
};

/*!
 * An OPTIONAL, independent orientation reference a tracking source may supply alongside (or
 * instead of) the ordinary position/orientation pose_prior that @ref
 * pose_metrics_evaluate_pose_with_prior already accepts. Unlike that prior (a single
 * possibly-stale predicted pose, needing tolerant thresholds to accommodate real motion since
 * the last visual fix), this is meant for a reference that stays fresh regardless of how long
 * ago the last visual fix was -- e.g. gyro-integrated IMU fusion -- so it can be checked with a
 * tight, staleness-blind, YAW-ONLY tolerance. Pitch/roll are deliberately never checked here:
 * they are gravity-truth already (see WMR_CONSTELLATION_GRAVITY_GATE_DEG) and must not be
 * conflated with this measurement.
 *
 * Motivating case (reverb-g2 project, T213, following on from patch 0074's device-side gate): a
 * near-180-degree yaw-flipped correspondence-search ghost, which a position/orientation-tolerant
 * pose_prior alone cannot catch without also rejecting genuine fast motion, and which the
 * existing CS_FLAG_MATCH_GRAVITY check cannot see at all (a rotation about the vertical axis
 * leaves the down vector fixed).
 */
struct pose_metrics_trusted_orientation
{
	//! Absolute orientation, in the SAME frame/convention as the `pose` argument being
	//! evaluated (i.e. camera-frame, Tcv_cam_device convention -- NOT world frame). Converting
	//! into this frame is the caller's job; this file has no notion of "world" at all.
	struct xrt_quat orientation;
	//! Unit vector, in that SAME frame, about which to isolate the yaw (twist) component via
	//! math_quat_decompose_swing_twist -- e.g. "world up as seen from this camera", the same
	//! vector CS_FLAG_MATCH_GRAVITY's own gravity_vector already is (see
	//! correspondence_search.h).
	struct xrt_vec3 up_vector;
	//! Maximum allowed |yaw error| in radians before a candidate is rejected/deprioritized.
	float yaw_threshold_rad;
};

struct pose_metrics_visible_led_info
{
	struct t_constellation_tracker_led *led;
	double led_radius_px;   //< Expected max size of the LED in pixels at that distance
	struct xrt_vec2 pos_px; //< Projected position of the LED (pixels)
	struct xrt_vec3 pos_m;  //< Projected physical position of the LED (metres)
	double facing_dot;      //< Dot product between LED and camera
	struct t_blob *matched_blob;
};

struct pose_metrics_blob_match_info
{
	struct pose_metrics_visible_led_info visible_leds[MAX_OBJECT_LEDS];
	int num_visible_leds;

	bool all_led_ids_matched;
	uint32_t matched_blobs;
	uint32_t unmatched_blobs;

	double reprojection_error;
	struct pose_rect bounds;
};

void
pose_metrics_match_pose_to_blobs(const struct xrt_pose *pose,
                                 struct t_blob *blobs,
                                 int num_blobs,
                                 struct t_constellation_tracker_led_model *led_model,
                                 t_constellation_device_id_t device_id,
                                 struct camera_model *calib,
                                 struct pose_metrics_blob_match_info *match_info);

void
pose_metrics_evaluate_pose(struct pose_metrics *score,
                           const struct xrt_pose *pose,
                           struct t_blob *blobs,
                           int num_blobs,
                           struct t_constellation_tracker_led_model *leds_model,
                           t_constellation_device_id_t device_id,
                           struct camera_model *calib,
                           struct pose_rect *out_bounds,
                           const struct pose_metrics_trusted_orientation *trusted_orientation);

void
pose_metrics_evaluate_pose_with_prior(struct pose_metrics *score,
                                      const struct xrt_pose *pose,
                                      bool prior_must_match,
                                      struct xrt_pose *pose_prior,
                                      const struct xrt_vec3 *pos_error_thresh,
                                      const struct xrt_vec3 *rot_error_thresh,
                                      struct t_blob *blobs,
                                      int num_blobs,
                                      struct t_constellation_tracker_led_model *led_model,
                                      t_constellation_device_id_t device_id,
                                      struct camera_model *calib,
                                      struct pose_rect *out_bounds,
                                      const struct pose_metrics_trusted_orientation *trusted_orientation);

/*!
 * Compares whether new_score is a better pose than old_score.
 *
 * @param old_score The old score to compare against.
 * @param new_score The new score to compare against the old score.
 * @return true if the new score is a better pose than the old score, false otherwise.
 */
bool
pose_metrics_score_is_better_pose(struct pose_metrics *old_score, struct pose_metrics *new_score);

#ifdef __cplusplus
}
#endif
