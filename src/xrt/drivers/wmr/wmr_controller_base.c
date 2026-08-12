// Copyright 2020-2021, N Madsen.
// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2020-2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for WMR Controller.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_wmr
 */

#include "os/os_time.h"
#include "os/os_hid.h"

#include "math/m_mathinclude.h"
#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_vec2.h"
#include "math/m_predict.h"

#include "util/u_file.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_trace_marker.h"

#include "wmr_common.h"
#include "wmr_controller_base.h"
#include "wmr_config_key.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

//! Radial thumbstick deadzone, applied by the per-model packet parsers. Off by default.
DEBUG_GET_ONCE_FLOAT_OPTION(wmr_stick_deadzone, "WMR_STICK_DEADZONE", 0.0f)
#include <string.h>
#include <assert.h>
#include <errno.h>

#define WMR_TRACE(wcb, ...) U_LOG_XDEV_IFL_T(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_TRACE_HEX(wcb, ...) U_LOG_XDEV_IFL_T_HEX(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_DEBUG(wcb, ...) U_LOG_XDEV_IFL_D(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_DEBUG_HEX(wcb, ...) U_LOG_XDEV_IFL_D_HEX(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_INFO(wcb, ...) U_LOG_XDEV_IFL_I(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_WARN(wcb, ...) U_LOG_XDEV_IFL_W(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_ERROR(wcb, ...) U_LOG_XDEV_IFL_E(&wcb->base, wcb->log_level, __VA_ARGS__)

#define wmr_controller_hexdump_buffer(wcb, label, buf, length)                                                         \
	do {                                                                                                           \
		WMR_DEBUG(wcb, "%s", label);                                                                           \
		WMR_DEBUG_HEX(wcb, buf, length);                                                                       \
	} while (0);


static inline struct wmr_controller_base *
wmr_controller_base(struct xrt_device *p)
{
	return (struct wmr_controller_base *)p;
}

static void
receive_bytes(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t buf_size)
{
	if (buf_size < 1) {
		WMR_ERROR(wcb, "WMR Controller: Error receiving short packet");
		return;
	}

	switch (buffer[0]) {
	case WMR_MOTION_CONTROLLER_STATUS_MSG:
		os_mutex_lock(&wcb->data_lock);
		// Note: skipping msg type byte
		bool b = wcb->handle_input_packet(wcb, time_ns, &buffer[1], (size_t)buf_size - 1);
		os_mutex_unlock(&wcb->data_lock);

		if (!b) {
			WMR_ERROR(wcb, "WMR Controller: Failed handling message type: %02x, size: %i", buffer[0],
			          buf_size);
			wmr_controller_hexdump_buffer(wcb, "Controller Message", buffer, buf_size);
			return;
		}

		break;
	default: WMR_DEBUG(wcb, "WMR Controller: Unknown message type: %02x, size: %i", buffer[0], buf_size); break;
	}

	return;
}

static bool
wmr_controller_send_bytes(struct wmr_controller_base *wcb, const uint8_t *buffer, uint32_t buf_size)
{
	bool res = false;

	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		res = wmr_controller_connection_send_bytes(conn, buffer, buf_size);
	}
	os_mutex_unlock(&wcb->conn_lock);

	return res;
}

static int
wmr_controller_read_sync(struct wmr_controller_base *wcb, uint8_t *buffer, uint32_t buf_size, int timeout_ms)
{
	int res = -1;
	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		res = wmr_controller_connection_read_sync(conn, buffer, buf_size, timeout_ms);
	}
	os_mutex_unlock(&wcb->conn_lock);

	return res;
}

static int
wmr_controller_send_fw_cmd(struct wmr_controller_base *wcb,
                           const struct wmr_controller_fw_cmd *fw_cmd,
                           unsigned char response_code,
                           struct wmr_controller_fw_cmd_response *response)
{
	// comms timeout. Replies are usually in 10ms or so but the first can take longer
	const int timeout_ms = 250;
	const int timeout_ns = timeout_ms * U_TIME_1MS_IN_NS;
	int64_t timeout_start = os_monotonic_get_ns();
	int64_t timeout_end_ns = timeout_start + timeout_ns;

	if (!wmr_controller_send_bytes(wcb, fw_cmd->buf, sizeof(fw_cmd->buf))) {
		return -1;
	}

	do {
		int size = wmr_controller_read_sync(wcb, response->buf, sizeof(response->buf), timeout_ms);
		if (size == -1) {
			return -1;
		}

		if (size < 1) {
			// Ignore 0-byte reads (timeout) and try again
			continue;
		}

		if (response->buf[0] == response_code) {
			WMR_TRACE(wcb, "Controller fw read returned %d bytes", size);
			if (size != sizeof(response->buf) || (response->response.cmd_id_echo != fw_cmd->cmd.cmd_id)) {
				WMR_DEBUG(
				    wcb, "Unexpected fw response - size %d (expected %zu), cmd_id_echo %u != cmd_id %u",
				    size, sizeof(response->buf), response->response.cmd_id_echo, fw_cmd->cmd.cmd_id);
				return -1;
			}

			response->response.blk_remain = __le32_to_cpu(response->response.blk_remain);
			return size;
		}
	} while (os_monotonic_get_ns() < timeout_end_ns);

	WMR_WARN(wcb, "Controller fw read timed out after %u ms",
	         (unsigned int)((os_monotonic_get_ns() - timeout_start) / U_TIME_1MS_IN_NS));
	return -ETIMEDOUT;
}

/* Firmware commands ride the same HID stream as the HMD IMU and both controllers' input
 * packets on the tunnelled (headset-radio) transport, so a single command can easily lose
 * its reply to foreign traffic. One lost reply used to fail the whole controller for the
 * session; the reads are offset-addressed (the request carries the remaining byte count),
 * so resending is idempotent and a short retry loop absorbs the transient losses. */
static int
wmr_controller_send_fw_cmd_retry(struct wmr_controller_base *wcb,
                                 const struct wmr_controller_fw_cmd *fw_cmd,
                                 unsigned char response_code,
                                 struct wmr_controller_fw_cmd_response *response,
                                 int attempts)
{
	/* Linear backoff between attempts; no sleep after the final one. */
	const int64_t backoff_step_ns = U_TIME_1MS_IN_NS * 20;

	int ret = -1;
	for (int i = 0; i < attempts; i++) {
		ret = wmr_controller_send_fw_cmd(wcb, fw_cmd, response_code, response);
		if (ret >= 0) {
			return ret;
		}
		WMR_DEBUG(wcb, "Controller fw cmd failed (%d), attempt %d of %d", ret, i + 1, attempts);
		if (i + 1 < attempts) {
			os_nanosleep(backoff_step_ns * (i + 1));
		}
	}
	return ret;
}

XRT_MAYBE_UNUSED static int
wmr_read_fw_block(struct wmr_controller_base *d, uint8_t blk_id, uint8_t **out_data, size_t *out_size)
{
	struct wmr_controller_fw_cmd_response fw_cmd_response;

	uint8_t *data;
	uint8_t *data_pos;
	uint8_t *data_end;
	uint32_t data_size;
	uint32_t remain;

	struct wmr_controller_fw_cmd fw_cmd;
	memset(&fw_cmd, 0, sizeof(fw_cmd));

	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x02, blk_id, 0xffffffff);
	if (wmr_controller_send_fw_cmd_retry(d, &fw_cmd, 0x02, &fw_cmd_response, 3) < 0) {
		WMR_WARN(d, "Failed to read fw - cmd 0x02 failed to read header for block %d", blk_id);
		return -1;
	}
	if (fw_cmd_response.response.block_id_echo != blk_id) {
		WMR_WARN(d, "Failed to read fw - stale reply for block %u while reading block %u",
		         fw_cmd_response.response.block_id_echo, blk_id);
		return -1;
	}

	data_size = fw_cmd_response.response.blk_remain + fw_cmd_response.response.len;
	WMR_DEBUG(d, "FW header %d bytes, %u bytes in block", fw_cmd_response.response.len, data_size);
	if (data_size == 0)
		return -1;

	data = calloc(1, data_size + 1);
	if (!data) {
		return -1;
	}
	data[data_size] = '\0';

	remain = data_size;
	data_pos = data;
	data_end = data + data_size;

	uint8_t to_copy = fw_cmd_response.response.len;

	memcpy(data_pos, fw_cmd_response.response.data, to_copy);
	data_pos += to_copy;
	remain -= to_copy;

	int stale_replies = 0;
	while (remain > 0) {
		fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x02, blk_id, remain);

		/* Brief pacing between chunk requests. This used to be 10ms, which put a
		 * multi-kB config read at seconds per controller - a needlessly wide window
		 * for a transient transport error to land in (the read_sync timeout already
		 * waits for the reply). */
		os_nanosleep(U_TIME_1MS_IN_NS);
		if (wmr_controller_send_fw_cmd_retry(d, &fw_cmd, 0x02, &fw_cmd_response, 3) < 0) {
			WMR_WARN(d, "Failed to read fw - cmd 0x02 failed @ offset %zu", data_pos - data);
			free(data);
			return -1;
		}

		/* With retries in play, a late reply to an earlier request can arrive after
		 * the resend - and fw replies only echo the command id, so it would match.
		 * Validate against this request: we asked for `remain` bytes, so the reply
		 * must account for exactly that many. Discard anything stale instead of
		 * letting it corrupt the block. */
		if (fw_cmd_response.response.block_id_echo != blk_id ||
		    fw_cmd_response.response.blk_remain + fw_cmd_response.response.len != remain) {
			WMR_DEBUG(d, "Discarding stale fw reply (block %u, %u+%u bytes, wanted %u)",
			          fw_cmd_response.response.block_id_echo, fw_cmd_response.response.blk_remain,
			          fw_cmd_response.response.len, remain);
			if (++stale_replies > 3) {
				WMR_WARN(d, "Failed to read fw - too many stale replies @ offset %zu", data_pos - data);
				free(data);
				return -1;
			}
			continue;
		}
		stale_replies = 0;

		uint8_t to_copy = fw_cmd_response.response.len;
		if (data_pos + to_copy > data_end)
			to_copy = data_end - data_pos;

		WMR_DEBUG(d, "Read %d bytes @ offset %zu / %d", to_copy, data_pos - data, data_size);
		memcpy(data_pos, fw_cmd_response.response.data, to_copy);
		data_pos += to_copy;
		remain -= to_copy;
	}

	WMR_DEBUG(d, "Read %d-byte FW data block %d", data_size, blk_id);
	wmr_controller_hexdump_buffer(d, "Data block", data, data_size);

	*out_data = data;
	*out_size = data_size;

	return 0;
}

/*
 *
 * Config functions.
 *
 */
static bool
read_controller_fw_info(struct wmr_controller_base *wcb,
                        uint32_t *fw_revision,
                        uint16_t *calibration_size,
                        char serial_no[16])
{
	uint8_t *data = NULL;
	size_t data_size;
	int ret;

	/* FW block 0 contains the FW revision (offset 0x14, size 4) and
	 * calibration block size (offset 0x34 size 2) */
	ret = wmr_read_fw_block(wcb, 0x0, &data, &data_size);
	if (ret < 0 || data == NULL) {
		WMR_ERROR(wcb, "Failed to read FW info block 0");
		return false;
	}
	if (data_size < 0x36) {
		WMR_ERROR(wcb, "Failed to read FW info block 0 - too short");
		free(data);
		return false;
	}


	const unsigned char *tmp = data + 0x14;
	*fw_revision = read32(&tmp);
	tmp = data + 0x34;
	*calibration_size = read16(&tmp);

	free(data);

	/* FW block 3 contains the controller serial number at offset
	 * 0x84, size 16 bytes */
	ret = wmr_read_fw_block(wcb, 0x3, &data, &data_size);
	if (ret < 0 || data == NULL) {
		WMR_ERROR(wcb, "Failed to read FW info block 3");
		return false;
	}
	if (data_size < 0x94) {
		WMR_ERROR(wcb, "Failed to read FW info block 3 - too short");
		free(data);
		return false;
	}

	memcpy(serial_no, data + 0x84, 0x10);
	serial_no[16] = '\0';

	free(data);
	return true;
}

char *
build_cache_filename(char *serial_no)
{
	int outlen = strlen("controller-") + strlen(serial_no) + strlen(".json") + 1;
	char *out = malloc(outlen);
	int ret = snprintf(out, outlen, "controller-%s.json", serial_no);

	assert(ret <= outlen);
	(void)ret;

	// Make sure the filename is valid
	for (char *cur = out; *cur != '\0'; cur++) {
		if (!isalnum(*cur) && *cur != '.') {
			*cur = '_';
		}
	}

	return out;
}

static bool
read_calibration_cache(struct wmr_controller_base *wcb, char *cache_filename)
{
	FILE *f = u_file_open_file_in_config_dir_subpath("wmr", cache_filename, "r");
	uint8_t *buffer = NULL;

	if (f == NULL) {
		WMR_DEBUG(wcb, "Failed to open wmr/%s cache file or it doesn't exist.", cache_filename);
		return false;
	}

	// Read the file size to allocate a read buffer
	fseek(f, 0L, SEEK_END);
	size_t file_size = ftell(f);

	// Reset and read the data
	fseek(f, 0L, SEEK_SET);

	buffer = calloc(file_size + 1, sizeof(uint8_t));
	if (buffer == NULL) {
		goto fail;
	}
	buffer[file_size] = '\0';

	size_t ret = fread(buffer, sizeof(char), file_size, f);
	if (ret != file_size) {
		WMR_WARN(wcb, "Cache file wmr/%s failed to read %u bytes (got %u)", cache_filename, (int)file_size,
		         (int)ret);
		goto fail;
	}

	if (!wmr_controller_config_parse(&wcb->config, (char *)buffer, wcb->log_level)) {
		WMR_WARN(wcb, "Cache file wmr/%s contains invalid JSON. Ignoring", cache_filename);
		goto fail;
	}

	fclose(f);
	free(buffer);

	return true;

fail:
	if (buffer) {
		free(buffer);
	}
	fclose(f);
	return false;
}

static void
write_calibration_cache(struct wmr_controller_base *wcb, char *cache_filename, uint8_t *data, size_t data_size)
{
	FILE *f = u_file_open_file_in_config_dir_subpath("wmr", cache_filename, "w");
	if (f == NULL) {
		return;
	}

	size_t ret = fwrite(data, sizeof(char), data_size, f);
	if (ret != data_size) {
		fclose(f);
		return;
	}

	fclose(f);
}

static bool
read_controller_config(struct wmr_controller_base *wcb)
{
	unsigned char *config_json_block;
	int ret;
	uint32_t fw_revision;
	uint16_t calibration_size;
	char serial_no[16 + 1];

	if (!read_controller_fw_info(wcb, &fw_revision, &calibration_size, serial_no)) {
		return false;
	}

	WMR_INFO(wcb, "Reading configuration for controller serial %s. FW revision %x", serial_no, fw_revision);

#if 0
  /* WMR also reads block 0x14, which seems to have some FW revision info,
   * but we don't use it */
	// Read block 0x14
	ret = wmr_read_fw_block(wcb, 0x14, &data, &data_size);
	if (ret < 0 || data == NULL)
		return false;
	free(data);
	data = NULL;
#endif

	// Read config block
	WMR_INFO(wcb, "Reading %s controller config",
	         wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ? "left" : "right");

	// Check if we have it cached already
	char *cache_filename = build_cache_filename(serial_no);

	if (!read_calibration_cache(wcb, cache_filename)) {
		unsigned char *data = NULL;
		size_t data_size;

		ret = wmr_read_fw_block(wcb, 0x02, &data, &data_size);
		if (ret < 0 || data == NULL || data_size < 2) {
			free(cache_filename);
			return false;
		}

		/* De-obfuscate the JSON config */
		config_json_block = data + sizeof(uint16_t);
		for (unsigned int i = 0; i < data_size - sizeof(uint16_t); i++) {
			config_json_block[i] ^= wmr_config_key[i % sizeof(wmr_config_key)];
		}

		if (!wmr_controller_config_parse(&wcb->config, (char *)config_json_block, wcb->log_level)) {
			free(cache_filename);
			free(data);
			return false;
		}

		/* Write to the cache file (if it fails, ignore it, it's just a cache) */
		write_calibration_cache(wcb, cache_filename, config_json_block, data_size - sizeof(uint16_t));
		free(data);
	} else {
		WMR_DEBUG(wcb, "Read %s controller config from cache %s",
		          wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ? "left" : "right",
		          cache_filename);
	}
	free(cache_filename);

	WMR_DEBUG(wcb, "Parsed %d LED entries from controller calibration", wcb->config.led_count);

	return true;
}

static xrt_result_t
wmr_controller_base_get_tracked_pose(struct xrt_device *xdev,
                                     enum xrt_input_name name,
                                     int64_t at_timestamp_ns,
                                     struct xrt_space_relation *out_relation)
{
	DRV_TRACE_MARKER();

	struct wmr_controller_base *wcb = wmr_controller_base(xdev);

	// Variables needed for prediction.
	int64_t last_imu_timestamp_ns = 0;
	struct xrt_space_relation relation = {0};
	relation.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);


	struct xrt_pose pose = {{0, 0, 0, 1}, {0, 1.2, -0.5}};
	if (xdev->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		pose.position.x = -0.2;
	} else {
		pose.position.x = 0.2;
	}

	// Real position from the constellation tracker (WMR_CONSTELLATION_CONTROLLERS), if the last sample
	// is fresh. Orientation below always comes from the IMU fusion regardless -- never overwritten by
	// the constellation solve, which only ever contributes position. Falls back to the placeholder
	// above (and drops the POSITION_*_BIT flags, exactly like before this patch) if the controller
	// hasn't been seen by a camera recently, e.g. it's out of view or the feature is off -- a stale
	// position silently marked "tracked" would be worse than the honest placeholder.
	//
	// 2026-08-12: this used to read constellation.last_pose directly and fall back to the
	// placeholder as soon as that one sample went stale, which made the hand snap to a fixed spot
	// the moment tracking gapped. It now comes out of the relation history, which interpolates
	// between samples and predicts across the gaps -- the same thing rift and pssense do. The
	// freshness window below still applies, on the newest sample rather than on a single stored
	// one, because predicting indefinitely from an old sample is how you get a hand drifting off
	// on its own.
	//
	// The clock-domain worry the previous comment described (samples reading seconds AHEAD of
	// at_timestamp_ns) was re-measured on this build and is gone: the delta is a steady 30-37 ms,
	// i.e. the ordinary age of the last 30 Hz camera frame. Patch 0016's clock alignment fixed it.
	// The absolute value is kept anyway -- it costs nothing and the failure it guards against is
	// silent.
	static const int64_t WMR_CONSTELLATION_MAX_SAMPLE_AGE_NS = 200000000; // 200ms, ~6 frames at 30fps
	int64_t constellation_sample_age_ns = at_timestamp_ns - wcb->constellation.last_timestamp_ns;
	if (constellation_sample_age_ns < 0) {
		constellation_sample_age_ns = -constellation_sample_age_ns;
	}
	// Absolute value: the constellation tracker's blob-observation timestamps and Monado's own
	// at_timestamp_ns aren't guaranteed to be in the same clock domain (observed the sample's
	// timestamp reading several seconds AHEAD of at_timestamp_ns in testing) -- a plain "is it in the
	// past and recent" check let a frozen, stale sample pass as fresh for 6+ seconds straight because
	// the negative delta was trivially less than the threshold. Treat "suspiciously far in either
	// direction" as equally stale.
	if (wcb->constellation.tracker != NULL && wcb->constellation.sample_count > 0 &&
	    constellation_sample_age_ns < WMR_CONSTELLATION_MAX_SAMPLE_AGE_NS) {
		struct xrt_space_relation constellation_relation = XRT_SPACE_RELATION_ZERO;
		m_relation_history_get(wcb->constellation.relation_history, at_timestamp_ns,
		                       &constellation_relation);

		// Position only, deliberately: the orientation in the history is the constellation
		// solve's, and the IMU's (applied further down, under the lock) is better. Taking only
		// the position is what this patch has always done; the history changes where the
		// position comes from, not which parts of the pose the constellation gets to set.
		if ((constellation_relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) {
			pose.position = constellation_relation.pose.position;
			relation.relation_flags = (enum xrt_space_relation_flags)(
			    relation.relation_flags | XRT_SPACE_RELATION_POSITION_VALID_BIT |
			    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		}
	}
	relation.pose = pose;

	// Copy data while holding the lock.
	os_mutex_lock(&wcb->data_lock);
	relation.pose.orientation = wcb->fusion.rot;
	relation.angular_velocity = wcb->last_angular_velocity;
	last_imu_timestamp_ns = wcb->last_imu_timestamp_ns;
	os_mutex_unlock(&wcb->data_lock);

	// No prediction needed.
	if (at_timestamp_ns < last_imu_timestamp_ns) {
		*out_relation = relation;
	} else {
		int64_t prediction_ns = at_timestamp_ns - last_imu_timestamp_ns;
		double prediction_s = time_ns_to_s(prediction_ns);

		m_predict_relation(&relation, prediction_s, out_relation);
	}

	// Throttled log of what this function actually hands back to the app -- as opposed to
	// constellation_sample_store's log of the tracker's internal telemetry -- so 0017 can be verified
	// from the real output, not just the solver's own bookkeeping. ~once/sec at typical 90Hz polling.
	// EXPERIMENT 2026-08-11: the original throttle used a function-local `static` counter, which is ONE
	// instance shared across every wcb (left AND right controller both call this function), so it only
	// ever fired for whichever device's calls happened to land on the modulo boundary -- silently hiding
	// the other controller's output the whole time. Gate on this device's OWN sample_count instead.
	// 2026-08-12: this used to gate on sample_count % 30, which only fires while the sample counter
	// happens to be sitting on a multiple of 30 -- so it stayed completely silent whenever samples
	// stopped arriving, i.e. exactly when the delta is worth reading. Gate on this device's own call
	// count instead; sample_count > 0 still keeps it quiet until there is something to report.
	static uint64_t get_tracked_pose_call_count = 0;
	++get_tracked_pose_call_count;
	if (wcb->constellation.sample_count > 0 && get_tracked_pose_call_count % 90 == 0) {
		bool pos_tracked = (out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
		WMR_INFO(wcb,
		        "get_tracked_pose: pos=(%.3f, %.3f, %.3f) position_tracked=%s at_ts=%lld "
		        "last_sample_ts=%lld delta_ms=%.1f sample_count=%llu",
		        out_relation->pose.position.x, out_relation->pose.position.y,
		        out_relation->pose.position.z, pos_tracked ? "yes" : "no (placeholder)",
		        (long long)at_timestamp_ns, (long long)wcb->constellation.last_timestamp_ns,
		        (double)(at_timestamp_ns - wcb->constellation.last_timestamp_ns) / 1e6,
		        (unsigned long long)wcb->constellation.sample_count);
	}

	return XRT_SUCCESS;
}

void
wmr_controller_base_deinit(struct wmr_controller_base *wcb)
{
	DRV_TRACE_MARKER();

	if (wcb->constellation.tracker != NULL) {
		t_constellation_tracker_remove_device(wcb->constellation.tracker, wcb->constellation.device_id);
	}

	// After remove_device: the tracker reads this history through the device's tracking_source, so
	// it must not be freed while the tracker can still call in.
	m_relation_history_destroy(&wcb->constellation.relation_history);

	// Remove the variable tracking.
	u_var_remove_root(wcb);

	// Disconnect from the connection so we don't
	// receive any more callbacks
	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	wcb->wcc = NULL;
	os_mutex_unlock(&wcb->conn_lock);

	if (conn != NULL) {
		wmr_controller_connection_disconnect(conn);
	}

	os_mutex_destroy(&wcb->conn_lock);
	os_mutex_destroy(&wcb->data_lock);

	// Destroy the fusion.
	m_imu_3dof_close(&wcb->fusion);
}

void
wmr_controller_base_apply_stick_deadzone(struct xrt_vec2 *stick)
{
	const float deadzone = debug_get_float_option_wmr_stick_deadzone();
	if (deadzone <= 0.0f || deadzone >= 1.0f) {
		return;
	}

	const float mag = sqrtf(stick->x * stick->x + stick->y * stick->y);
	if (mag < deadzone) {
		stick->x = 0.0f;
		stick->y = 0.0f;
		return;
	}

	/* Rescale so the active range still spans 0..1. A full diagonal has mag > 1,
	 * where the rescale can push a component past 1 - clamp per component. */
	const float rescale = (mag - deadzone) / (1.0f - deadzone) / mag;
	stick->x = fminf(fmaxf(stick->x * rescale, -1.0f), 1.0f);
	stick->y = fminf(fmaxf(stick->y * rescale, -1.0f), 1.0f);
}

/*
 *
 * 'Exported' functions.
 *
 */

bool
wmr_controller_base_init(struct wmr_controller_base *wcb,
                         struct wmr_controller_connection *conn,
                         enum xrt_device_type controller_type,
                         enum u_logging_level log_level,
                         u_device_destroy_function_t destroy_fn)
{
	DRV_TRACE_MARKER();

	wcb->log_level = log_level;
	wcb->wcc = conn;
	wcb->receive_bytes = receive_bytes;
	wcb->constellation.device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;

	if (controller_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "WMR Left Controller");
		/* TODO: use proper serial from read_controller_config()? */
		snprintf(wcb->base.serial, XRT_DEVICE_NAME_LEN, "Left Controller");
	} else {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "WMR Right Controller");
		/* TODO: use proper serial from read_controller_config()? */
		snprintf(wcb->base.serial, XRT_DEVICE_NAME_LEN, "Right Controller");
	}

	// Set all functions.
	u_device_populate_function_pointers(&wcb->base, wmr_controller_base_get_tracked_pose, destroy_fn);

	wcb->base.name = XRT_DEVICE_WMR_CONTROLLER;
	wcb->base.device_type = controller_type;
	wcb->base.supported.orientation_tracking = true;
	wcb->base.supported.position_tracking = false;
	wcb->base.supported.hand_tracking = false;

	m_imu_3dof_init(&wcb->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	if (os_mutex_init(&wcb->conn_lock) != 0 || os_mutex_init(&wcb->data_lock) != 0) {
		WMR_ERROR(wcb, "WMR Controller: Failed to init mutex!");
		return false;
	}

	u_var_add_root(wcb, wcb->base.str, true);

	/* Send init commands */
	struct wmr_controller_fw_cmd fw_cmd = {
	    0,
	};
	struct wmr_controller_fw_cmd_response fw_cmd_response;

	/* Zero command. Reinits controller internal state */
	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x0, 0, 0);
	if (wmr_controller_send_fw_cmd(wcb, &fw_cmd, 0x06, &fw_cmd_response) < 0) {
		return false;
	}

	/* Quiesce/restart controller tasks */
	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x04, 0xc1, 0x02);
	if (wmr_controller_send_fw_cmd(wcb, &fw_cmd, 0x06, &fw_cmd_response) < 0) {
		return false;
	}

	// Read config file from controller
	if (!read_controller_config(wcb)) {
		return false;
	}

	wmr_config_precompute_transforms(&wcb->config.sensors, NULL);

	/* Enable the status reports, IMU and control status reports */
	const unsigned char wmr_controller_status_enable_cmd[64] = {0x06, 0x03, 0x01, 0x00, 0x02};
	wmr_controller_send_bytes(wcb, wmr_controller_status_enable_cmd, sizeof(wmr_controller_status_enable_cmd));
	const unsigned char wmr_controller_imu_on_cmd[64] = {0x06, 0x03, 0x02, 0xe1, 0x02};
	wmr_controller_send_bytes(wcb, wmr_controller_imu_on_cmd, sizeof(wmr_controller_imu_on_cmd));

	return true;
}

//! LED physical size isn't in the WMR calibration JSON (only position + normal) -- this is a
//! reasonable estimate for the G2 controller ring, not a measured value. Revisit if 0016's
//! telemetry shows a poor reprojection fit.
#define WMR_CONSTELLATION_LED_RADIUS_M 0.003f
#define WMR_CONSTELLATION_LED_VISIBILITY_ANGLE DEG_TO_RAD(75)

//! @ref t_constellation_tracker_device::push_constellation_tracker_sample. Feeds @ref
//! wmr_controller_base.constellation.relation_history, which is what both the reported position and
//! the tracker's own prior are read from. The last_* fields below are kept for the debug GUI.
//! Orientation keeps coming exclusively from the IMU fusion, as always.
static void
constellation_sample_store(struct t_constellation_tracker_device *device, struct t_constellation_tracker_sample *sample)
{
	struct wmr_controller_base *wcb =
	    container_of(device, struct wmr_controller_base, constellation.device);

	// Position only: the constellation solve's orientation is not used for anything (the IMU's is
	// better), but the history needs both bits set or every reader treats the entry as invalid.
	// The orientation stored here is therefore the solve's, and deliberately never read back out
	// as the device's orientation -- see wmr_controller_base_get_tracked_pose.
	struct xrt_space_relation relation = {
	    .pose = sample->pose,
	    .relation_flags = (enum xrt_space_relation_flags)(
	        XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT),
	};
	m_relation_history_push(wcb->constellation.relation_history, &relation, sample->timestamp_ns);

	wcb->constellation.last_pose = sample->pose;
	wcb->constellation.last_timestamp_ns = sample->timestamp_ns;
	wcb->constellation.last_metrics = sample->metrics;
	wcb->constellation.sample_count++;

	// MEASUREMENT 2026-08-12: how far apart are the constellation solve's orientation and the IMU
	// fusion's? The solve is bistable on this headset (two clusters 0.19 m apart for a motionless
	// controller), and the IMU is the only independent evidence available to break the tie -- but
	// only if the two live in the same frame. The fusion's yaw is gravity-referenced with an
	// arbitrary heading, so agreement is NOT a given and feeding it in blind could reject the true
	// pose. MEASURED: the gravity axis alone disagrees by 104-161 deg, and bimodally, matching the
	// two position clusters -- so this is not the yaw question, the two orientations are in
	// different frames entirely (the LED model is in the controller's calibration frame, the fusion
	// in the IMU's, which has its own pose inside that model). Resolving that fixed transform from
	// the factory calibration is what an IMU-backed prior needs first; it is NOT a drop-in.
	if (wcb->constellation.sample_count % 15 == 0) {
		struct xrt_quat imu_rot;
		os_mutex_lock(&wcb->data_lock);
		imu_rot = wcb->fusion.rot;
		os_mutex_unlock(&wcb->data_lock);

		struct xrt_quat solve_inv, delta;
		math_quat_invert(&sample->pose.orientation, &solve_inv);
		math_quat_rotate(&imu_rot, &solve_inv, &delta);

		// Angle of the rotation that takes one to the other, and the same for gravity's direction
		// alone (roll/pitch only, immune to the yaw question).
		float angle_deg = (float)(2.0 * acos(fmin(1.0, fabs((double)delta.w))) * 180.0 / M_PI);
		struct xrt_vec3 down = {0.f, -1.f, 0.f};
		struct xrt_vec3 down_imu, down_solve;
		math_quat_rotate_vec3(&imu_rot, &down, &down_imu);
		math_quat_rotate_vec3(&sample->pose.orientation, &down, &down_solve);
		float dot = down_imu.x * down_solve.x + down_imu.y * down_solve.y + down_imu.z * down_solve.z;
		float gravity_deg = (float)(acos(fmax(-1.f, fmin(1.f, dot))) * 180.0 / M_PI);

		WMR_DEBUG(wcb, "constellation vs IMU: full=%.1f deg gravity_axis=%.1f deg", angle_deg,
		          gravity_deg);
	}

	// Throttled to ~2/s at 30fps so this is readable in the log without needing the debug GUI.
	if (wcb->constellation.sample_count % 15 == 0) {
		WMR_INFO(wcb,
		        "constellation sample #%llu: pos=(%.3f, %.3f, %.3f) matched_blobs=%u "
		        "visible_leds=%u reproj_err_px=%.2f",
		        (unsigned long long)wcb->constellation.sample_count, sample->pose.position.x,
		        sample->pose.position.y, sample->pose.position.z, sample->metrics.matched_blob_count,
		        sample->metrics.visible_led_count, sample->metrics.reprojection_error);
	}
}

/*!
 * The prior the constellation tracker asks for before it goes looking for this controller: "where
 * do you think you are right now?". Answered from the controller's own recent constellation
 * samples, predicted forward to @p when_ns by @ref m_relation_history.
 *
 * This is what stops the tracker picking a differently-oriented pose that happens to fit the same
 * handful of blobs. Measured on the G2 2026-08-12, before this existed: a controller lying
 * motionless on a desk had a p99 step of 0.41-0.44 m between consecutive samples, with healthy
 * metrics on the bad ones (8 matched blobs at 0.06 px reprojection error, 45 cm away from the
 * truth). With few LEDs visible several poses fit almost perfectly, so no threshold on the
 * tracker's own quality metrics can separate them -- only outside information can, and this is it.
 */
static void
constellation_tracking_source_get_tracked_pose(struct t_constellation_tracker_tracking_source *tracking_source,
                                               int64_t when_ns,
                                               struct xrt_space_relation *out_relation)
{
	struct wmr_controller_base *wcb =
	    container_of(tracking_source, struct wmr_controller_base, constellation.tracking_source);

	m_relation_history_get(wcb->constellation.relation_history, when_ns, out_relation);
}

void
wmr_controller_base_add_to_constellation_tracker(struct wmr_controller_base *wcb,
                                                 struct t_constellation_tracker *tracker)
{
	if (tracker == NULL || wcb->config.led_count == 0) {
		return;
	}

	for (int i = 0; i < wcb->config.led_count; i++) {
		wcb->constellation.leds[i] = (struct t_constellation_tracker_led){
		    .position = wcb->config.leds[i].pos,
		    .normal = wcb->config.leds[i].norm,
		    .radius_m = WMR_CONSTELLATION_LED_RADIUS_M,
		    .visibility_angle = WMR_CONSTELLATION_LED_VISIBILITY_ANGLE,
		    .id = i,
		};
	}

	wcb->constellation.led_model = (struct t_constellation_tracker_led_model){
	    .leds = wcb->constellation.leds,
	    .led_count = (size_t)wcb->config.led_count,
	    // No custom ring-occlusion model yet -- pose_metrics.c's built-in facing_dot/visibility_angle
	    // check (same one pssense relies on) is a reasonable first pass. See t_constellation.h's own
	    // note that WMR's inward-facing ring LEDs are exactly the motivating case for this callback
	    // if the generic check turns out not to be enough.
	    .compute_led_visibility = NULL,
	};

	wcb->constellation.device.push_constellation_tracker_sample = constellation_sample_store;

	if (wcb->constellation.relation_history == NULL) {
		m_relation_history_create(&wcb->constellation.relation_history);
	}
	wcb->constellation.tracking_source.get_tracked_pose = constellation_tracking_source_get_tracked_pose;

	struct t_constellation_tracker_device_params params = {
	    .led_model = wcb->constellation.led_model,
	    .tracking_source = &wcb->constellation.tracking_source,
	};

	int ret = t_constellation_tracker_add_device(tracker, &params, &wcb->constellation.device,
	                                             &wcb->constellation.device_id);
	if (ret != 0) {
		WMR_WARN(wcb, "Failed to register with constellation tracker, code %d -- stays orientation-only", ret);
		wcb->constellation.device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
		return;
	}

	wcb->constellation.tracker = tracker;

	// Adopt the tracker's own tracking origin (XRT_TRACKING_TYPE_CONSTELLATION), same as pssense/PSVR2
	// does for the same kind of camera-driven controller tracking. This is what makes the fused
	// position in wmr_controller_base_get_tracked_pose meaningful in the space graph instead of
	// composing against the generic placeholder origin u_builder_setup_tracking_origins hands out to
	// XRT_TRACKING_TYPE_NONE devices -- see that function for the y=1.3 placeholder this replaces.
	wcb->base.tracking_origin = t_constellation_tracker_get_tracking_origin(tracker);
	wcb->base.supported.position_tracking = true;

	u_var_add_gui_header(wcb, NULL, "Constellation tracking (telemetry only)");
	u_var_add_ro_u64(wcb, &wcb->constellation.sample_count, "constellation.sample_count");
	u_var_add_ro_vec3_f32(wcb, &wcb->constellation.last_pose.position, "constellation.position_m");
	u_var_add_ro_u32(wcb, &wcb->constellation.last_metrics.matched_blob_count, "constellation.matched_blobs");
	u_var_add_ro_u32(wcb, &wcb->constellation.last_metrics.visible_led_count, "constellation.visible_leds");
	u_var_add_ro_f64(wcb, &wcb->constellation.last_metrics.reprojection_error, "constellation.reprojection_error_px");

	WMR_INFO(wcb, "Registered with constellation tracker (%d LEDs, device id %d) -- telemetry only, no effect "
	              "on the output pose yet",
	         wcb->config.led_count, wcb->constellation.device_id);
}
