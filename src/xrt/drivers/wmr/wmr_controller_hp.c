// Copyright 2020-2021, N Madsen.
// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2020-2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for WMR Controllers.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_wmr
 */
#include "math/m_api.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>

#include "wmr_controller.h"

// WMR_CONTROLLER_LEFT_YAW_GYRO_INVERT (2026-08-17, T206/T207 live derivation): the LEFT G2
// controller's calibrated gyro Y-component has the wrong sign, and ONLY that axis -- pitch and
// roll are correct on both hands, and the RIGHT controller's gyro is correct on all three axes.
// See the long derivation comment at the use site below (wmr_controller_hp_packet_parse) for the
// measured numbers and the proof that this canNOT be fixed downstream as an output-orientation
// rotation/reflection composed in wmr_controller_base_get_tracked_pose (a single flipped body
// axis is a reflection with an odd sign-flip count, which no rotation OR reflection conjugation
// of the delivered quaternion can reproduce without also disturbing the other two, already-good,
// axes -- that is exactly why WMR_CONTROLLER_ORIENT_FIX's plain conjugate made things worse
// instead of better). The only mathematically sound fix is to correct the sign at the point the
// bad axis enters the fusion filter, before it gets integrated. Default off; LEFT hand only.
DEBUG_GET_ONCE_BOOL_OPTION(wmr_controller_left_yaw_gyro_invert, "WMR_CONTROLLER_LEFT_YAW_GYRO_INVERT", false)

#define WMR_TRACE(ctrl, ...) U_LOG_XDEV_IFL_T(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_TRACE_HEX(ctrl, ...) U_LOG_XDEV_IFL_T_HEX(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_DEBUG(ctrl, ...) U_LOG_XDEV_IFL_D(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_DEBUG_HEX(ctrl, ...) U_LOG_XDEV_IFL_D_HEX(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_INFO(ctrl, ...) U_LOG_XDEV_IFL_I(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_WARN(ctrl, ...) U_LOG_XDEV_IFL_W(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)
#define WMR_ERROR(ctrl, ...) U_LOG_XDEV_IFL_E(&ctrl->base.base, ctrl->base.log_level, __VA_ARGS__)

#ifdef XRT_DOXYGEN
#define WMR_PACKED
#else
#define WMR_PACKED __attribute__((packed))
#endif

/*!
 * Indices in input list of each input.
 */
enum wmr_controller_hp_input_index
{
	WMR_CONTROLLER_INDEX_MENU_CLICK,
	WMR_CONTROLLER_INDEX_HOME_CLICK,
	WMR_CONTROLLER_INDEX_SQUEEZE_CLICK,
	WMR_CONTROLLER_INDEX_SQUEEZE_VALUE,
	WMR_CONTROLLER_INDEX_TRIGGER_VALUE,
	WMR_CONTROLLER_INDEX_THUMBSTICK_CLICK,
	WMR_CONTROLLER_INDEX_THUMBSTICK,
	WMR_CONTROLLER_INDEX_GRIP_POSE,
	WMR_CONTROLLER_INDEX_AIM_POSE,
	WMR_CONTROLLER_INDEX_X_A_CLICK,
	WMR_CONTROLLER_INDEX_Y_B_CLICK,
	/* keep as last: */
	WMR_CONTROLLER_INDEX_COUNT
};

#define SET_INPUT(wcb, INDEX, NAME)                                                                                    \
	(wcb->base.inputs[WMR_CONTROLLER_INDEX_##INDEX].name = XRT_INPUT_G2_CONTROLLER_##NAME)

/*
 *
 * Bindings
 *
 */

static struct xrt_binding_input_pair touch_inputs[19] = {
    {XRT_INPUT_TOUCH_X_CLICK, XRT_INPUT_G2_CONTROLLER_X_CLICK},
    {XRT_INPUT_TOUCH_X_TOUCH, XRT_INPUT_G2_CONTROLLER_X_CLICK},
    {XRT_INPUT_TOUCH_Y_CLICK, XRT_INPUT_G2_CONTROLLER_Y_CLICK},
    {XRT_INPUT_TOUCH_Y_TOUCH, XRT_INPUT_G2_CONTROLLER_Y_CLICK},
    {XRT_INPUT_TOUCH_MENU_CLICK, XRT_INPUT_G2_CONTROLLER_MENU_CLICK},
    {XRT_INPUT_TOUCH_MENU_CLICK, XRT_INPUT_G2_CONTROLLER_HOME_CLICK},
    {XRT_INPUT_TOUCH_A_CLICK, XRT_INPUT_G2_CONTROLLER_A_CLICK},
    {XRT_INPUT_TOUCH_A_TOUCH, XRT_INPUT_G2_CONTROLLER_A_CLICK},
    {XRT_INPUT_TOUCH_B_CLICK, XRT_INPUT_G2_CONTROLLER_B_CLICK},
    {XRT_INPUT_TOUCH_B_TOUCH, XRT_INPUT_G2_CONTROLLER_B_CLICK},
    {XRT_INPUT_TOUCH_SYSTEM_CLICK, XRT_INPUT_G2_CONTROLLER_MENU_CLICK},
    {XRT_INPUT_TOUCH_SYSTEM_CLICK, XRT_INPUT_G2_CONTROLLER_HOME_CLICK},
    {XRT_INPUT_TOUCH_SQUEEZE_VALUE, XRT_INPUT_G2_CONTROLLER_SQUEEZE_VALUE},
    {XRT_INPUT_TOUCH_TRIGGER_TOUCH, XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE},
    {XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE},
    {XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_G2_CONTROLLER_THUMBSTICK_CLICK},
    {XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_G2_CONTROLLER_THUMBSTICK},
    {XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_G2_CONTROLLER_GRIP_POSE},
    {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_G2_CONTROLLER_AIM_POSE},
};

static struct xrt_binding_output_pair touch_outputs[1] = {
    {XRT_OUTPUT_NAME_TOUCH_HAPTIC, XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC},
};

static struct xrt_binding_input_pair simple_inputs[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_G2_CONTROLLER_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_G2_CONTROLLER_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_G2_CONTROLLER_AIM_POSE},
};

static struct xrt_binding_output_pair simple_outputs[1] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC},
};

/*
 * Remap for the native /interaction_profiles/microsoft/motion_controller profile
 * (xrt_device name XRT_DEVICE_WMR_CONTROLLER, per bindings.json's "monado_device"). This
 * driver self-identifies as XRT_DEVICE_HP_REVERB_G2_CONTROLLER (see set_controller_props()
 * below) and its inputs are named XRT_INPUT_G2_CONTROLLER_*, not XRT_INPUT_WMR_* - without
 * this table, oxr_input.c's get_binding() finds neither a name match nor a binding_profiles
 * fallback for that profile (profile->xname != xdev->name && xbp == NULL) and silently drops
 * every action bound to it. Discovered 2026-08-06: a WMR thumbstick binding for video seek
 * never fired, and this is why - not just the new action, ALL microsoft/motion_controller
 * bindings (grip pose, squeeze, quit) were unreachable on real G2 hardware, masked because
 * the app happened to still get pose/grip/select through the khr/simple_controller fallback
 * (simple_inputs below) since hello_xr suggests bindings for that profile too.
 */
static struct xrt_binding_input_pair wmr_inputs[7] = {
    {XRT_INPUT_WMR_MENU_CLICK, XRT_INPUT_G2_CONTROLLER_MENU_CLICK},
    {XRT_INPUT_WMR_SQUEEZE_CLICK, XRT_INPUT_G2_CONTROLLER_SQUEEZE_CLICK},
    {XRT_INPUT_WMR_TRIGGER_VALUE, XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE},
    {XRT_INPUT_WMR_THUMBSTICK_CLICK, XRT_INPUT_G2_CONTROLLER_THUMBSTICK_CLICK},
    {XRT_INPUT_WMR_THUMBSTICK, XRT_INPUT_G2_CONTROLLER_THUMBSTICK},
    {XRT_INPUT_WMR_GRIP_POSE, XRT_INPUT_G2_CONTROLLER_GRIP_POSE},
    {XRT_INPUT_WMR_AIM_POSE, XRT_INPUT_G2_CONTROLLER_AIM_POSE},
};

static struct xrt_binding_output_pair wmr_outputs[1] = {
    {XRT_OUTPUT_NAME_WMR_HAPTIC, XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC},
};

static struct xrt_binding_profile binding_profiles[3] = {
    {
        .name = XRT_DEVICE_TOUCH_CONTROLLER,
        .inputs = touch_inputs,
        .input_count = ARRAY_SIZE(touch_inputs),
        .outputs = touch_outputs,
        .output_count = ARRAY_SIZE(touch_outputs),
    },
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs,
        .input_count = ARRAY_SIZE(simple_inputs),
        .outputs = simple_outputs,
        .output_count = ARRAY_SIZE(simple_outputs),
    },
    {
        .name = XRT_DEVICE_WMR_CONTROLLER,
        .inputs = wmr_inputs,
        .input_count = ARRAY_SIZE(wmr_inputs),
        .outputs = wmr_outputs,
        .output_count = ARRAY_SIZE(wmr_outputs),
    },
};

/* OG WMR Controller inputs struct */
struct wmr_controller_hp_input
{
	// buttons clicked
	bool menu;
	bool home;
	bool bt_pairing;
	bool squeeze_click; // Squeeze click reported on full squeeze

	// X/Y/A/B buttons
	bool x_a;
	bool y_b;

	float trigger;
	float squeeze;

	struct
	{
		bool click;
		struct xrt_vec2 values;
	} thumbstick;

	uint8_t battery;

	struct
	{
		uint64_t timestamp_ticks;
		struct xrt_vec3 acc;
		struct xrt_vec3 gyro;
		int32_t temperature;
		//! True when this packet carried all-zero raw IMU counts, see the parse function.
		bool zeroed;
	} imu;
};
#undef WMR_PACKED

/* HP WMR Controller device struct */
struct wmr_controller_hp
{
	struct wmr_controller_base base;

	//! The last decoded package of IMU and button data
	struct wmr_controller_hp_input last_inputs;

	//! Zeroed-IMU (idle) packets seen, for the throttled log below.
	uint64_t zeroed_imu_count;

	//! True once last_inputs.battery has been set at least once by a real packet -- guards
	//! get_battery_status() from reporting a fake 0% before the controller has said anything.
	bool has_battery_sample;
};

/*
 *
 * WMR Motion Controller protocol helpers
 *
 */

static inline void
vec3_from_wmr_controller_accel(const int32_t sample[3], struct xrt_vec3 *out_vec)
{
	// Reverb G1 observation: 1g is approximately 490,000.
	// @todo: Confirm the scale is correct

	out_vec->x = (float)sample[0] / (98000 / 2);
	out_vec->y = (float)sample[1] / (98000 / 2);
	out_vec->z = (float)sample[2] / (98000 / 2);
}


static inline void
vec3_from_wmr_controller_gyro(const int32_t sample[3], struct xrt_vec3 *out_vec)
{
	// @todo: Confirm the scale is correct
	out_vec->x = (float)sample[0] * 0.00001f;
	out_vec->y = (float)sample[1] * 0.00001f;
	out_vec->z = (float)sample[2] * 0.00001f;
}

static bool
wmr_controller_hp_packet_parse(struct wmr_controller_hp *ctrl, const unsigned char *buffer, size_t len)
{
	struct wmr_controller_hp_input *last_input = &ctrl->last_inputs;
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(ctrl);

	if (len != 44) {
		U_LOG_IFL_E(wcb->log_level, "WMR Controller: unexpected message length: %zd", len);
		return false;
	}

	const unsigned char *p = buffer;

	// Read buttons
	uint8_t buttons = read8(&p);
	last_input->thumbstick.click = buttons & 0x01;
	last_input->home = buttons & 0x02;
	last_input->menu = buttons & 0x04;
	last_input->squeeze_click = buttons & 0x08; // squeeze-click
	last_input->bt_pairing = buttons & 0x20;

	// Read thumbstick coordinates (12 bit resolution)
	int16_t stick_x = read8(&p);
	uint8_t nibbles = read8(&p);
	stick_x += ((nibbles & 0x0F) << 8);
	int16_t stick_y = (nibbles >> 4);
	stick_y += (read8(&p) << 4);

	last_input->thumbstick.values.x = (float)(stick_x - 0x07FF) / 0x07FF;
	if (last_input->thumbstick.values.x > 1.0f) {
		last_input->thumbstick.values.x = 1.0f;
	}

	last_input->thumbstick.values.y = (float)(stick_y - 0x07FF) / 0x07FF;
	if (last_input->thumbstick.values.y > 1.0f) {
		last_input->thumbstick.values.y = 1.0f;
	}

	wmr_controller_base_apply_stick_deadzone(&last_input->thumbstick.values);

	// Read trigger value (0x00 - 0xFF)
	last_input->trigger = (float)read8(&p) / 0xFF;

	/* On OG these are touchpad values, but on HP it's
	 * squeeze value and A_X/B_Y click */
	last_input->squeeze = (float)read8(&p) / 0xFF;

	buttons = read8(&p);
	last_input->x_a = buttons & 0x02;
	last_input->y_b = buttons & 0x01;

	uint8_t new_battery = read8(&p);
	if (!ctrl->has_battery_sample || new_battery != last_input->battery) {
		// Scale unverified -- see the long comment in wmr_controller_hp_get_battery_status()
		// below. Logged on every CHANGE (not every packet) specifically so a real
		// charge/discharge cycle can be correlated against this byte later.
		WMR_INFO(ctrl, "Controller battery raw byte: %u -> %u", last_input->battery, new_battery);
	}
	last_input->battery = new_battery;
	ctrl->has_battery_sample = true;

	int32_t acc[3];
	acc[0] = read24(&p); // x
	acc[1] = read24(&p); // y
	acc[2] = read24(&p); // z
	vec3_from_wmr_controller_accel(acc, &last_input->imu.acc);
	math_matrix_3x3_transform_vec3(&wcb->config.sensors.accel.mix_matrix, &last_input->imu.acc,
	                               &last_input->imu.acc);
	math_vec3_accum(&wcb->config.sensors.accel.bias_offsets, &last_input->imu.acc);
	math_quat_rotate_vec3(&wcb->config.sensors.transforms.P_oxr_acc.orientation, &last_input->imu.acc,
	                      &last_input->imu.acc);

	U_LOG_IFL_T(wcb->log_level, "Accel [m/s^2] : %f",
	            sqrtf(last_input->imu.acc.x * last_input->imu.acc.x +
	                  last_input->imu.acc.y * last_input->imu.acc.y +
	                  last_input->imu.acc.z * last_input->imu.acc.z));


	last_input->imu.temperature = read16(&p);

	int32_t gyro[3];
	gyro[0] = read24(&p);
	gyro[1] = read24(&p);
	gyro[2] = read24(&p);

	// Idle detection, on the RAW counts and before calibration. ~30 s after the controller stops
	// moving, its firmware keeps the 44-byte packet stream alive (buttons, battery, timestamps all
	// valid) but zeroes the six IMU fields. The calibration pipeline below then manufactures
	// sensor data out of nothing: 0 * mix_matrix + bias_offsets = the factory bias vector, a small
	// CONSTANT -- measured 2026-08-13 as |accel| frozen at exactly 0.171 m/s^2 on one controller
	// and 0.121 on the other across every idle packet, values that are impossible for a real
	// resting accelerometer (it must read ~9.81) and are precisely each device's own offsets.
	// The matching fake constant gyro (0.006-0.021 rad/s) integrated into the 20-70 deg/min
	// "drift" chased across two sessions. Six exact zeros cannot come from a live sensor, so this
	// is unambiguous. The flag makes handle_input_packet skip fusion for these packets; buttons
	// keep working.
	last_input->imu.zeroed = acc[0] == 0 && acc[1] == 0 && acc[2] == 0 && //
	                         gyro[0] == 0 && gyro[1] == 0 && gyro[2] == 0;

	vec3_from_wmr_controller_gyro(gyro, &last_input->imu.gyro);
	math_matrix_3x3_transform_vec3(&wcb->config.sensors.gyro.mix_matrix, &last_input->imu.gyro,
	                               &last_input->imu.gyro);
	math_vec3_accum(&wcb->config.sensors.gyro.bias_offsets, &last_input->imu.gyro);
	math_quat_rotate_vec3(&wcb->config.sensors.transforms.P_oxr_gyr.orientation, &last_input->imu.gyro,
	                      &last_input->imu.gyro);

	// WMR_CONTROLLER_LEFT_YAW_GYRO_INVERT (2026-08-17, T206/T207): live labeled A/B/C motion
	// captures (WMR_CONTROLLER_CALIBRATION_LOG=1; still 10s -> 5x pure pitch -> still ->
	// 5x pure roll -> still -> 5x pure yaw -> still, RIGHT then LEFT controller, same session,
	// same wearer, same physical motions) measured the dominant post-calibration gyro axis
	// (this exact .imu.gyro vector, after mix_matrix + bias + P_oxr_gyr above) per phase:
	//
	//               RIGHT (known-good, wearer-confirmed)   LEFT (wearer-confirmed wrong)
	//   PITCH  ->    +X   (rms 2.09 vs 0.48/0.56 other)     +X   (rms 1.77 vs 0.18/0.29 other)
	//   ROLL   ->    -Z   (rms 1.79 vs 0.52/0.52 other)     -Z   (rms 1.59 vs 0.48/0.68 other)
	//   YAW    ->    +Y   (rms 1.70 vs 0.57/0.63 other)     -Y   (rms 1.57 vs 0.55/0.64 other)
	//
	// Pitch and roll already match the target OpenXR grip convention (physical pitch about
	// +X, physical roll about the forward axis Z) IDENTICALLY on both hands, sign included --
	// R = identity is the right answer there, matching the live wearer report that the right
	// controller needs no fix at all. Yaw is the ONE axis that differs between hands, by a
	// pure sign flip, with pitch/roll on that same hand otherwise unaffected.
	//
	// That is NOT something any output-side orientation transform can correct, and this is
	// provable, not just empirically likely. Any candidate fix of the existing A/B menu's
	// shape -- q_out = q_imu * R, or R * q_imu, or q_imu^-1, for some fixed quaternion R --
	// composes only PROPER rotations (determinant +1 by construction: quaternions can only
	// ever represent rotations, never reflections) with q_imu (also a proper rotation). The
	// determinant of a product is the product of the determinants, so any such q_out is
	// related to q_imu by an overall map whose determinant is always +1*+1 = +1. But the
	// map this bug actually needs -- leave X and Z exactly alone, flip only Y -- has
	// determinant (+1)*(-1)*(+1) = -1: a REFLECTION. No composition of proper rotations can
	// ever equal a reflection (+1 can never equal -1). This is exactly why
	// WMR_CONTROLLER_ORIENT_FIX's plain conjugate (patch 0061, itself a proper rotation:
	// q^-1 has determinant +1 too) made the symptom WORSE ("rotates about unseen
	// intermediate axes") instead of fixing it -- it was mathematically incapable of
	// reproducing this fault no matter how it's tuned, and so is every other knob in this
	// file's existing A/B menu. The only place a single-axis sign defect like this CAN be
	// fixed is before it becomes part of a rotation at all -- i.e. on the raw vector, here.
	//
	// It also explains WHY only yaw is affected: m_imu_3dof's accelerometer/gravity
	// correction (M_IMU_3DOF_USE_GRAVITY_DUR_20MS, wcb->fusion) continuously re-anchors pitch
	// and roll against gravity every update, so a wrong-signed gyro axis feeding those two
	// gets corrected out almost immediately. Yaw has no such reference -- gravity cannot
	// observe heading -- so a sign error on the yaw-carrying axis integrates uncorrected
	// forever. The fix therefore has to happen HERE, before m_imu_3dof_update ever sees the
	// sample, not as a post-hoc rotation of its output.
	//
	// NOT yet hardware-root-caused (still open whether the true defect is in the factory
	// gyro.mix_matrix, the bias_offsets, or P_oxr_gyr.orientation for this specific unit --
	// none of those raw values are logged today) -- this is the smallest, most direct,
	// numerically-verified compensating fix at the one point in the pipeline where the
	// measured symptom (post-calibration .imu.gyro) already matches the target exactly once
	// Y is negated. Left hand only; right hand's pipeline is untouched. Default off, for a
	// live A/B.
	if (wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER &&
	    debug_get_bool_option_wmr_controller_left_yaw_gyro_invert()) {
		last_input->imu.gyro.y = -last_input->imu.gyro.y;
		// MEASURED NEGATIVE RESULT, do not re-add (2026-08-17, T207): negating the
		// accel Y here too -- which the frame-reconciliation algebra suggested as
		// the "coherent" diag(1,-1,1) on both sensors -- made the left controller
		// PRECESS: a 3D figure-8 wound up whole extra turns instead of closing
		// ("sigue pegando vueltas completas"), while the right's closed exactly.
		// A flipped gravity reference fighting a correct gyro never settles. The
		// empirical truth is gyro-only: with just the gyro negation the wearer
		// reported all three axes rotating correctly ("los ejes parecen estar
		// bien"), leaving only a constant heading offset -- which is the separate
		// no-absolute-yaw-reference problem, not a frame error.
	}

	uint32_t prev_ticks = last_input->imu.timestamp_ticks & UINT32_C(0xFFFFFFFF);

	// Write the new ticks value into the lower half of timestamp_ticks
	last_input->imu.timestamp_ticks &= (UINT64_C(0xFFFFFFFF) << 32u);
	last_input->imu.timestamp_ticks += (uint32_t)read32(&p);

	if ((last_input->imu.timestamp_ticks & UINT64_C(0xFFFFFFFF)) < prev_ticks) {
		// Timer overflow, so increment the upper half of timestamp_ticks
		last_input->imu.timestamp_ticks += (UINT64_C(0x1) << 32u);
	}

	/* Todo: More decoding here
	    read16(&p); // Unknown. Seems to depend on controller orientation (probably mag)
	    read32(&p); // Unknown.
	    read16(&p); // Unknown. Device state, etc.
	    read16(&p);
	    read16(&p);
	*/

	return true;
}

static bool
handle_input_packet(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t buf_size)
{
	struct wmr_controller_hp *ctrl = (struct wmr_controller_hp *)(wcb);

	bool b = wmr_controller_hp_packet_parse(ctrl, buffer, buf_size);
	if (b && ctrl->last_inputs.imu.zeroed) {
		// Valid packet, but its IMU fields are the firmware's idle zeros (see the parse function)
		// -- feeding them to the fusion integrates a fake constant rotation. Skip the fusion AND
		// leave last_imu_timestamp_ns alone: staleness then grows past the prediction cap in
		// get_tracked_pose, which freezes the reported pose with zero velocities. Between the two,
		// a resting controller finally just sits still.
		ctrl->zeroed_imu_count++;
		if (ctrl->zeroed_imu_count == 1 || (ctrl->zeroed_imu_count % 1000) == 0) {
			WMR_DEBUG(ctrl, "idle (zeroed) IMU packet -- fusion paused [%" PRIu64 " so far]",
			          ctrl->zeroed_imu_count);
		}
	} else if (b) {
		m_imu_3dof_update(&wcb->fusion,
		                  ctrl->last_inputs.imu.timestamp_ticks * WMR_MOTION_CONTROLLER_NS_PER_TICK,
		                  &ctrl->last_inputs.imu.acc, &ctrl->last_inputs.imu.gyro);

		wcb->last_imu_timestamp_ns = time_ns;
		wcb->last_angular_velocity = ctrl->last_inputs.imu.gyro;
	}

	return b;
}

static xrt_result_t
wmr_controller_hp_update_inputs(struct xrt_device *xdev)
{
	DRV_TRACE_MARKER();

	struct wmr_controller_hp *ctrl = (struct wmr_controller_hp *)(xdev);
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(xdev);

	os_mutex_lock(&wcb->data_lock);

	struct xrt_input *xrt_inputs = xdev->inputs;
	struct wmr_controller_hp_input *cur_inputs = &ctrl->last_inputs;

	xrt_inputs[WMR_CONTROLLER_INDEX_MENU_CLICK].value.boolean = cur_inputs->menu;
	xrt_inputs[WMR_CONTROLLER_INDEX_HOME_CLICK].value.boolean = cur_inputs->home;
	xrt_inputs[WMR_CONTROLLER_INDEX_X_A_CLICK].value.boolean = cur_inputs->x_a;
	xrt_inputs[WMR_CONTROLLER_INDEX_Y_B_CLICK].value.boolean = cur_inputs->y_b;
	// squeeze_click is the parsed button bit; assigning the analog squeeze float here made
	// any non-zero grip pressure read as a click.
	xrt_inputs[WMR_CONTROLLER_INDEX_SQUEEZE_CLICK].value.boolean = cur_inputs->squeeze_click;
	xrt_inputs[WMR_CONTROLLER_INDEX_SQUEEZE_VALUE].value.vec1.x = cur_inputs->squeeze;
	xrt_inputs[WMR_CONTROLLER_INDEX_TRIGGER_VALUE].value.vec1.x = cur_inputs->trigger;
	xrt_inputs[WMR_CONTROLLER_INDEX_THUMBSTICK_CLICK].value.boolean = cur_inputs->thumbstick.click;
	xrt_inputs[WMR_CONTROLLER_INDEX_THUMBSTICK].value.vec2 = cur_inputs->thumbstick.values;

	// Without a timestamp OpenXR reports lastChangeTime == 0 for every WMR action.
	for (uint32_t i = 0; i < xdev->input_count; i++) {
		xrt_inputs[i].timestamp = (int64_t)wcb->last_imu_timestamp_ns;
	}

	os_mutex_unlock(&wcb->data_lock);

	return XRT_SUCCESS;
}

static xrt_result_t
wmr_controller_hp_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct wmr_controller_hp *ctrl = (struct wmr_controller_hp *)(xdev);
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(xdev);

	os_mutex_lock(&wcb->data_lock);
	bool have_sample = ctrl->has_battery_sample;
	uint8_t raw = ctrl->last_inputs.battery;
	os_mutex_unlock(&wcb->data_lock);

	if (!have_sample) {
		// Never received an input packet yet (controller off/out of range/not yet connected)
		// -- report "no data", not a fake 0%. Same spirit as u_device_ni_get_battery_status's
		// not-implemented stub, just scoped to "not yet known" instead of "never known".
		*out_present = false;
		return XRT_SUCCESS;
	}

	*out_present = true;
	// The G2 controller runs on 2x AAA batteries with no onboard charging circuit -- there is
	// nothing to report here, unlike the Rift Touch or pssense controllers next to this driver
	// in the tree.
	*out_charging = false;

	// UNVERIFIED SCALE, said plainly so a caller does not trust this more than it has earned.
	// `raw` is the uint8_t this driver has parsed out of the controller's 44-byte input report
	// since the very first version of this file (wmr_controller_hp_packet_parse above) -- it
	// has simply never been surfaced anywhere past Monado's own debug-variable system (u_var,
	// i.e. only visible with XRT_DEBUG_GUI=1 open and a human looking at the right panel)
	// until this function. Nothing found so far -- not this tree, not the Windows HID capture
	// in docs/09-oasis-driver-re.md, not any WMR community writeup -- documents whether this
	// byte is already a 0-100 percentage, a raw ADC reading, or something else again. Treating
	// it as a plain 0-255 range (out_charge = raw / 255) is the least-committal reading, not a
	// confirmed calibration. wmr_controller_hp_packet_parse logs every time this byte CHANGES,
	// specifically so the real scale can be worked out later by watching it across a real
	// charge/discharge cycle instead of guessed at again.
	*out_charge = (float)raw / 255.0f;

	return XRT_SUCCESS;
}

static void
wmr_controller_hp_destroy(struct xrt_device *xdev)
{
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(xdev);

	wmr_controller_base_deinit(wcb);
	free(wcb);
}

struct wmr_controller_base *
wmr_controller_hp_create(struct wmr_controller_connection *conn,
                         enum xrt_device_type controller_type,
                         enum u_logging_level log_level)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
	struct wmr_controller_hp *ctrl =
	    U_DEVICE_ALLOCATE(struct wmr_controller_hp, flags, WMR_CONTROLLER_INDEX_COUNT, 1);
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(ctrl);

	if (!wmr_controller_base_init(wcb, conn, controller_type, log_level, wmr_controller_hp_destroy)) {
		wmr_controller_hp_destroy(&wcb->base);
		return NULL;
	}

	wcb->handle_input_packet = handle_input_packet;

	// Only set those we want to overwrite.
	wcb->base.update_inputs = wmr_controller_hp_update_inputs;
	wcb->base.get_battery_status = wmr_controller_hp_get_battery_status;
	wcb->base.supported.battery_status = true;
	wcb->base.name = XRT_DEVICE_HP_REVERB_G2_CONTROLLER;

	if (controller_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "HP Reverb G2 Left Controller");
	} else {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "HP Reverb G2 Right Controller");
	}

	SET_INPUT(wcb, MENU_CLICK, MENU_CLICK);
	SET_INPUT(wcb, HOME_CLICK, HOME_CLICK);
	SET_INPUT(wcb, SQUEEZE_CLICK, SQUEEZE_CLICK);
	SET_INPUT(wcb, SQUEEZE_VALUE, SQUEEZE_VALUE);
	SET_INPUT(wcb, TRIGGER_VALUE, TRIGGER_VALUE);
	SET_INPUT(wcb, THUMBSTICK_CLICK, THUMBSTICK_CLICK);
	SET_INPUT(wcb, THUMBSTICK, THUMBSTICK);
	SET_INPUT(wcb, GRIP_POSE, GRIP_POSE);
	SET_INPUT(wcb, AIM_POSE, AIM_POSE);
	if (controller_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		SET_INPUT(wcb, X_A_CLICK, X_CLICK);
		SET_INPUT(wcb, Y_B_CLICK, Y_CLICK);
	} else {
		SET_INPUT(wcb, X_A_CLICK, A_CLICK);
		SET_INPUT(wcb, Y_B_CLICK, B_CLICK);
	}

	for (uint32_t i = 0; i < wcb->base.input_count; i++) {
		wcb->base.inputs[i].active = true;
	}

	ctrl->last_inputs.imu.timestamp_ticks = 0;

	// Must match what the binding profiles and bindings.json reference
	// (XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC) or the haptic action can never resolve.
	wcb->base.outputs[0].name = XRT_OUTPUT_NAME_G2_CONTROLLER_HAPTIC;

	wcb->base.binding_profiles = binding_profiles;
	wcb->base.binding_profile_count = ARRAY_SIZE(binding_profiles);

	u_var_add_bool(wcb, &ctrl->last_inputs.menu, "input.menu");
	u_var_add_bool(wcb, &ctrl->last_inputs.home, "input.home");
	u_var_add_bool(wcb, &ctrl->last_inputs.bt_pairing, "input.bt_pairing");
	u_var_add_bool(wcb, &ctrl->last_inputs.squeeze_click, "input.squeeze.click");
	u_var_add_f32(wcb, &ctrl->last_inputs.squeeze, "input.squeeze.value");
	u_var_add_f32(wcb, &ctrl->last_inputs.trigger, "input.trigger");
	u_var_add_u8(wcb, &ctrl->last_inputs.battery, "input.battery");
	u_var_add_bool(wcb, &ctrl->last_inputs.thumbstick.click, "input.thumbstick.click");
	u_var_add_f32(wcb, &ctrl->last_inputs.thumbstick.values.x, "input.thumbstick.values.x");
	u_var_add_f32(wcb, &ctrl->last_inputs.thumbstick.values.y, "input.thumbstick.values.y");
	if (controller_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		u_var_add_bool(wcb, &ctrl->last_inputs.x_a, "input.x");
		u_var_add_bool(wcb, &ctrl->last_inputs.y_b, "input.y");
	} else {
		u_var_add_bool(wcb, &ctrl->last_inputs.x_a, "input.a");
		u_var_add_bool(wcb, &ctrl->last_inputs.y_b, "input.b");
	}

	u_var_add_ro_vec3_f32(wcb, &ctrl->last_inputs.imu.acc, "imu.acc");
	u_var_add_ro_vec3_f32(wcb, &ctrl->last_inputs.imu.gyro, "imu.gyro");
	u_var_add_i32(wcb, &ctrl->last_inputs.imu.temperature, "imu.temperature");

	return wcb;
}
