// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A IMU fusion specially made for 3dof devices.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#include "util/u_misc.h"
#include "util/u_var.h"

#include "math/m_api.h"
#include "math/m_filter_fifo.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h"

#include <stdio.h>
#include <assert.h>

#define DUR_1S_IN_NS (1000 * 1000 * 1000)
#define DUR_300MS_IN_NS (300 * 1000 * 1000)
#define DUR_20MS_IN_NS (20 * 1000 * 1000)


void
m_imu_3dof_init(struct m_imu_3dof *f, int flags)
{
	U_ZERO(f);
	f->rot.w = 1.0f;

	m_ff_vec3_f32_alloc(&f->word_accel_ff, 1000);
	m_ff_vec3_f32_alloc(&f->gyro_ff, 1000);

	f->flags = flags;
	f->gyro_bias.auto_enabled = (flags & M_IMU_3DOF_USE_GYRO_BIAS_AUTO) != 0;
}

void
m_imu_3dof_reset(struct m_imu_3dof *f)
{
	struct m_ff_vec3_f32 *aff = f->word_accel_ff;
	struct m_ff_vec3_f32 *gff = f->gyro_ff;
	int flags = f->flags;

	U_ZERO(f);
	f->rot.w = 1.0f;
	f->word_accel_ff = aff;
	f->gyro_ff = gff;
	f->flags = flags;
}

void
m_imu_3dof_close(struct m_imu_3dof *f)
{
	m_ff_vec3_f32_free(&f->word_accel_ff);
	m_ff_vec3_f32_free(&f->gyro_ff);
}

void
m_imu_3dof_add_vars(struct m_imu_3dof *f, void *root, const char *prefix)
{
	char tmp[512];
	snprintf(tmp, sizeof(tmp), "%sff.world_accel", prefix);
	u_var_add_ro_ff_vec3_f32(root, f->word_accel_ff, tmp);
	snprintf(tmp, sizeof(tmp), "%sff.gyro", prefix);
	u_var_add_ro_ff_vec3_f32(root, f->gyro_ff, tmp);

	snprintf(tmp, sizeof(tmp), "%slast.timestamp_ns", prefix);
	u_var_add_ro_u64(root, &f->last.timestamp_ns, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro", prefix);
	u_var_add_ro_vec3_f32(root, &f->last.gyro, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro_length", prefix);
	u_var_add_ro_f32(root, &f->last.gyro_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro_biased_length", prefix);
	u_var_add_ro_f32(root, &f->last.gyro_biased_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.accel", prefix);
	u_var_add_ro_vec3_f32(root, &f->last.accel, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.accel_length", prefix);
	u_var_add_ro_f32(root, &f->last.accel_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.delta_ms", prefix);
	u_var_add_ro_f64(root, &f->last.delta_ms, tmp);

	snprintf(tmp, sizeof(tmp), "%sgrav.level_timestamp_ns", prefix);
	u_var_add_ro_u64(root, &f->grav.level_timestamp_ns, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.error_axis", prefix);
	u_var_add_ro_vec3_f32(root, &f->grav.error_axis, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.error_angle", prefix);
	u_var_add_ro_f32(root, &f->grav.error_angle, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.is_accel", prefix);
	u_var_add_bool(root, &f->grav.is_accel, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.is_rotating", prefix);
	u_var_add_bool(root, &f->grav.is_rotating, tmp);

	snprintf(tmp, sizeof(tmp), "%sgyro_bias.value", prefix);
	u_var_add_ro_vec3_f32(root, &f->gyro_bias.value, tmp);
	snprintf(tmp, sizeof(tmp), "%sgyro_bias.manually_fire", prefix);
	u_var_add_bool(root, &f->gyro_bias.manually_fire, tmp);
	snprintf(tmp, sizeof(tmp), "%sgyro_bias.auto_fire_count", prefix);
	u_var_add_ro_u32(root, &f->gyro_bias.auto_fire_count, tmp);
}

static void
gravity_correction(struct m_imu_3dof *f,
                   uint64_t timestamp_ns,
                   const struct xrt_vec3 *accel,
                   const struct xrt_vec3 *gyro,
                   double dt,
                   float gyro_length)
{
	uint64_t dur_ns = 0;
	if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) {
		dur_ns = DUR_20MS_IN_NS;
	} else if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_300MS) {
		dur_ns = DUR_300MS_IN_NS;
	} else {
		return;
	}

	const float gravity_tolerance = .9f;
	const float gyro_tolerance = .1f;
	const float min_tilt_error = 0.05f;
	const float max_tilt_error = 0.01f;

	/*
	 * If the device is within tolerance levels, count this
	 * as the device is level and add to the counter otherwise
	 * reset the counter and start over.
	 */

	float accel_length = m_vec3_len(*accel);
	bool is_accel = fabsf(accel_length - 9.82f) >= gravity_tolerance;
	bool is_rotating = gyro_length >= gyro_tolerance;
	if (is_accel || is_rotating) {
		f->grav.level_timestamp_ns = timestamp_ns;
	}
	f->grav.is_accel = is_accel;
	f->grav.is_rotating = is_rotating;

	/*
	 * Device has been level for long enough, grab mean from the
	 * accelerometer filter queue (last n values) and use for correction.
	 */
	uint64_t level_ns = f->grav.level_timestamp_ns + dur_ns;
	if (level_ns < timestamp_ns) {
		// Reset the timepoint
		f->grav.level_timestamp_ns = timestamp_ns;

		struct xrt_vec3 accel_mean;
		m_ff_vec3_f32_filter(f->word_accel_ff,      // Filter
		                     timestamp_ns - dur_ns, // Start time
		                     timestamp_ns,          // End time
		                     &accel_mean);          // Results
		if ((m_vec3_len(accel_mean) - 9.82f) < gravity_tolerance) {
			/*
			 * Calculate a cross product between what the device
			 * thinks is up and what gravity indicates is down.
			 * The values are optimized of what we would get out
			 * from the cross product.
			 */
			struct xrt_vec3 tilt = {
			    accel_mean.z,
			    0,
			    -accel_mean.x,
			};

			tilt = m_vec3_normalize(tilt);
			accel_mean = m_vec3_normalize(accel_mean);

			struct xrt_vec3 up = {0, 1.0f, 0};
			float tilt_angle = m_vec3_angle(up, accel_mean);

			if (tilt_angle > max_tilt_error) {
				f->grav.error_angle = tilt_angle;
				f->grav.error_axis = tilt;
			}
		}
	}

	if (f->grav.error_angle > min_tilt_error) {
		// Correct 180° over 5 seconds, when moving.
		float max_radians = (float)M_PI * (float)dt / 5;
		// Correct 180° over 60 seconds, when stationary.
		float min_radians = (float)M_PI * (float)dt / 60;

		/*
		 * We're treating 0.5 * gyro_length as a unitless scale factor.
		 * Tested in a headset, 0.5 felt nice.
		 */
		float correction_radians = 0.5f * gyro_length * max_radians;
		// Clamp to the range [min_radians, max_radians]
		correction_radians = fmaxf(min_radians, correction_radians);
		correction_radians = fminf(max_radians, correction_radians);
		// Do not exceed the remaining error to correct for
		correction_radians = -fminf(correction_radians, f->grav.error_angle);

		// Update how much is left.
		f->grav.error_angle += correction_radians;

		// Perform the correction.
		struct xrt_quat corr_quat;
		struct xrt_quat old_orient;
		math_quat_from_angle_vector(correction_radians, &f->grav.error_axis, &corr_quat);
		old_orient = f->rot;
		math_quat_rotate(&corr_quat, &old_orient, &f->rot);
	}
}

/*!
 * Decide whether the device is being held still, and if it has been for long enough, ask for a
 * fresh bias estimate. See M_IMU_3DOF_USE_GYRO_BIAS_AUTO for why this is needed at all.
 *
 * "Still" is judged from the RAW gyro, deliberately: while the device is motionless the gyro is
 * reading its own bias and nothing else, which is the whole basis of the estimate. The threshold
 * therefore has to sit above the bias being measured and well below real motion. Measured on this
 * hardware the bias is ~0.021 rad/s (72 deg/min, the worse of the two controllers) and deliberate
 * hand motion is upwards of 0.5 rad/s, so 0.10 rad/s separates them with an order of magnitude of
 * room on the low side.
 *
 * The accelerometer has to agree: a device in free fall or being accelerated in a straight line
 * can show near-zero rotation while very much not being still.
 */
static void
gyro_bias_auto(struct m_imu_3dof *f, uint64_t timestamp_ns, float gyro_length, float accel_length)
{
	if (!f->gyro_bias.auto_enabled) {
		return;
	}

	//! Above the bias being measured, far below intentional motion. See the comment above.
	const float STILL_GYRO_RAD_S = 0.10f;
	//! Gravity only, to within a small margin. MATH_GRAVITY_M_S2 is 9.8066.
	const float STILL_ACCEL_TOLERANCE = 0.5f;
	//! Long enough that the 300 ms averaging window is entirely inside the still stretch.
	const uint64_t STILL_REQUIRED_NS = 1000 * 1000 * 1000ULL;
	//! While it stays still, keep refreshing at this interval -- bias moves with temperature.
	const uint64_t REESTIMATE_EVERY_NS = 2000 * 1000 * 1000ULL;

	bool still = gyro_length < STILL_GYRO_RAD_S &&
	             fabsf(accel_length - MATH_GRAVITY_M_S2) < STILL_ACCEL_TOLERANCE;

	if (!still) {
		f->gyro_bias.still_since_ns = 0;
		return;
	}

	if (f->gyro_bias.still_since_ns == 0) {
		f->gyro_bias.still_since_ns = timestamp_ns;
		return;
	}

	if (timestamp_ns - f->gyro_bias.still_since_ns < STILL_REQUIRED_NS) {
		return;
	}

	if (f->gyro_bias.last_auto_ns != 0 && timestamp_ns - f->gyro_bias.last_auto_ns < REESTIMATE_EVERY_NS) {
		return;
	}

	f->gyro_bias.last_auto_ns = timestamp_ns;
	f->gyro_bias.auto_fire_count++;
	f->gyro_bias.manually_fire = true; // reuse the existing estimator rather than duplicating it

	// Say so out loud. A silent estimator is indistinguishable from one that never runs, and
	// this whole change is only worth anything if it actually fires -- the first measurement
	// after adding it was inconclusive precisely because there was no way to tell.
	U_LOG_I("gyro bias auto-estimate #%u fired: bias now (%.5f, %.5f, %.5f) rad/s (%.2f deg/min)",
	        f->gyro_bias.auto_fire_count, f->gyro_bias.value.x, f->gyro_bias.value.y,
	        f->gyro_bias.value.z,
	        (double)m_vec3_len(f->gyro_bias.value) * 180.0 / M_PI * 60.0);
}

static void
gyro_biasing(struct m_imu_3dof *f, uint64_t timestamp_ns)
{
	if (!f->gyro_bias.manually_fire) {
		return;
	}

	f->gyro_bias.manually_fire = false;

	uint64_t dur_ns = DUR_300MS_IN_NS;

	struct xrt_vec3 gyro_mean = XRT_VEC3_ZERO;
	m_ff_vec3_f32_filter(f->gyro_ff,            // Filter
	                     timestamp_ns - dur_ns, // Start time
	                     timestamp_ns,          // End time
	                     &gyro_mean);           // Results

	f->gyro_bias.value = gyro_mean;
}

void
m_imu_3dof_update(struct m_imu_3dof *f,
                  uint64_t timestamp_ns,
                  const struct xrt_vec3 *accel,
                  const struct xrt_vec3 *gyro)
{
	//! Skip the first sample.
	if (f->state == M_IMU_3DOF_STATE_START) {
		f->state = M_IMU_3DOF_STATE_RUNNING;
		f->last.timestamp_ns = timestamp_ns;
		return;
	}

	// This code assumes all timestamps makes some forward progress.
	assert(timestamp_ns >= f->last.timestamp_ns);

	f->last.gyro = *gyro;
	f->last.accel = *accel;

	struct xrt_vec3 world_accel = {0};
	math_quat_rotate_vec3(&f->rot, accel, &world_accel);

	uint64_t diff = timestamp_ns - f->last.timestamp_ns;
	double dt = (double)diff / DUR_1S_IN_NS;

	f->last.delta_ms = dt * 1000.0f;
	f->last.timestamp_ns = timestamp_ns;

	m_ff_vec3_f32_push(f->word_accel_ff, &world_accel, timestamp_ns);
	m_ff_vec3_f32_push(f->gyro_ff, gyro, timestamp_ns);

	struct xrt_vec3 gyro_biased = m_vec3_sub(*gyro, f->gyro_bias.value);
	float gyro_biased_length = m_vec3_len(gyro_biased);
	float gyro_length = m_vec3_len(*gyro);
	float accel_length = m_vec3_len(*accel);

	f->last.accel_length = accel_length;
	f->last.gyro_length = gyro_length;
	f->last.gyro_biased_length = gyro_biased_length;


	if (gyro_biased_length > 0.0001f) {
#if 0
		math_quat_integrate_velocity(&f->rot, gyro, dt, &f->rot);
#else
		struct xrt_vec3 rot_axis = {
		    gyro_biased.x / gyro_biased_length,
		    gyro_biased.y / gyro_biased_length,
		    gyro_biased.z / gyro_biased_length,
		};

		float rot_angle = gyro_biased_length * (float)dt;

		struct xrt_quat delta_orient;
		math_quat_from_angle_vector(rot_angle, &rot_axis, &delta_orient);

		math_quat_rotate(&f->rot, &delta_orient, &f->rot);
#endif
	}

	// Gravity correction.
	gravity_correction(f, timestamp_ns, accel, &gyro_biased, dt, gyro_biased_length);

	// Gyro bias calculations. The automatic trigger runs first and simply asks the existing
	// estimator to run, so both paths share one implementation.
	gyro_bias_auto(f, timestamp_ns, gyro_length, accel_length);
	gyro_biasing(f, timestamp_ns);

	/*
	 * Mitigate drift due to floating point
	 * inprecision with quat multiplication.
	 */
	math_quat_normalize(&f->rot);
}
