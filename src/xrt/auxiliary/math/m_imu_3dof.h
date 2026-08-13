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

#pragma once

#include "xrt/xrt_defines.h"


#ifdef __cplusplus
extern "C" {
#endif


#define M_IMU_3DOF_USE_GRAVITY_DUR_300MS (1 << 0)
#define M_IMU_3DOF_USE_GRAVITY_DUR_20MS (1 << 1)

/*!
 * Estimate the gyroscope bias automatically whenever the device is held still, instead of only
 * when something sets @ref m_imu_3dof.gyro_bias.manually_fire -- which, in practice, is a
 * checkbox in the debug GUI that nothing in a real session ever touches.
 *
 * Without it the residual bias is integrated forever and the orientation simply rotates away.
 * Measured on an HP Reverb G2's controllers, 2026-08-12, both lying untouched on a desk, two
 * independent windows agreeing within 1%: the LEFT controller drifted 72 deg/min and the right
 * 20 deg/min, 93% and 63% of steps monotonic. Gravity correction cannot save this -- the
 * accelerometer establishes vertical, so pitch and roll are pulled back but YAW has no absolute
 * reference at all and runs unbounded. That is exactly the symptom the wearer reports.
 */
#define M_IMU_3DOF_USE_GYRO_BIAS_AUTO (1 << 2)


struct m_ff_vec3_f32;

enum m_imu_3dof_state
{
	M_IMU_3DOF_STATE_START = 0,
	M_IMU_3DOF_STATE_RUNNING = 1,
};

struct m_imu_3dof
{
	struct xrt_quat rot; //!< Orientation

	struct
	{
		uint64_t timestamp_ns;
		struct xrt_vec3 gyro;  //!< Angular velocity
		struct xrt_vec3 accel; //!< Acceleration
		double delta_ms;
		float accel_length;
		float gyro_length;
		float gyro_biased_length;
	} last;

	enum m_imu_3dof_state state;

	int flags;

	// Filter fifos for accelerometer and gyroscope.
	struct m_ff_vec3_f32 *word_accel_ff;
	struct m_ff_vec3_f32 *gyro_ff;

	// gravity correction
	struct
	{
		uint64_t level_timestamp_ns;
		struct xrt_vec3 error_axis;
		float error_angle;
		bool is_accel;
		bool is_rotating;
	} grav;

	// gyro bias correction
	struct
	{
		struct xrt_vec3 value;
		bool manually_fire;

		//! When the device was first seen to be still in the current stretch, 0 if moving.
		uint64_t still_since_ns;
		//! When the estimate last ran, so a long still stretch re-estimates periodically.
		uint64_t last_auto_ns;
		//! How many times the automatic path has fired, for the GUI and for measuring.
		uint32_t auto_fire_count;
		//! Estimates folded into @ref value so far; the first one is taken whole, see gyro_biasing.
		uint32_t estimate_count;
	} gyro_bias;
};

void
m_imu_3dof_init(struct m_imu_3dof *f, int flags);

void
m_imu_3dof_reset(struct m_imu_3dof *f);

void
m_imu_3dof_close(struct m_imu_3dof *f);

void
m_imu_3dof_add_vars(struct m_imu_3dof *f, void *root, const char *prefix);

void
m_imu_3dof_update(struct m_imu_3dof *f,
                  uint64_t timestamp_ns,
                  const struct xrt_vec3 *accel,
                  const struct xrt_vec3 *gyro);


#ifdef __cplusplus
}
#endif
