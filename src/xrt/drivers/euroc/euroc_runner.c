// Copyright 2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Play EuRoC datasets and track them with the SLAM tracker.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup drv_euroc
 */

#include "euroc_driver.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "tracking/t_tracking.h"
#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"
#include "xrt/xrt_tracking.h"

#if !defined(XRT_FEATURE_SLAM)

void
euroc_run_dataset(const char *euroc_path,
                  const char *slam_config,
                  const char *output_path,
                  const volatile bool *should_exit)
{}

#else

//! How often to ask the tracker for a pose during a batch run, in Hz. The loop below polls at
//! 5 Hz, which is enough to notice that tracking stopped but useless for anything else: PREDICTION
//! only ever runs inside get_tracked_pose, so prediction.csv comes out ~34x sparser than a real
//! session's ~170 Hz and cannot characterise the SLAM_PRED_* knobs at all. 0 restores the original
//! 5 Hz-only behaviour.
DEBUG_GET_ONCE_NUM_OPTION(slam_batch_poll_hz, "SLAM_BATCH_POLL_HZ", 170)

static struct euroc_player_config *
make_euroc_player_config(const char *euroc_path)
{
	struct euroc_player_config *ep_config = U_TYPED_CALLOC(struct euroc_player_config);
	euroc_player_fill_default_config_for(ep_config, euroc_path);

	// Override config to be friendlier for CLI runs unless they were explicitly provided
	if (getenv("EUROC_LOG") == NULL) {
		ep_config->log_level = U_LOGGING_INFO;
	}
	if (getenv("EUROC_PLAY_FROM_START") == NULL) {
		ep_config->playback.play_from_start = true;
	}
	if (getenv("EUROC_PRINT_PROGRESS") == NULL) {
		ep_config->playback.print_progress = true;
	}
	if (getenv("EUROC_USE_SOURCE_TS") == NULL) {
		ep_config->playback.use_source_ts = true;
	}
	if (getenv("EUROC_MAX_SPEED") == NULL) {
		ep_config->playback.max_speed = true;
	}

	return ep_config;
}

static struct t_slam_tracker_config *
make_slam_tracker_config(const char *slam_config, const char *output_path)
{
	struct t_slam_tracker_config *st_config = U_TYPED_CALLOC(struct t_slam_tracker_config);
	t_slam_fill_default_config(st_config);

	// Override config to be friendlier for CLI runs unless they were explicitly provided
	if (getenv("SLAM_LOG") == NULL) {
		st_config->log_level = U_LOGGING_INFO;
	}
	if (getenv("SLAM_SUBMIT_FROM_START") == NULL) {
		st_config->submit_from_start = true;
	}
	if (getenv("SLAM_PREDICTION_TYPE") == NULL) {
		st_config->prediction = SLAM_PRED_NONE;
	}
	if (getenv("SLAM_WRITE_CSVS") == NULL) {
		st_config->write_csvs = true;
	}

	st_config->slam_config = slam_config;
	st_config->csv_path = output_path;

	return st_config;
}

void
euroc_run_dataset(const char *euroc_path,
                  const char *slam_config,
                  const char *output_path,
                  const volatile bool *should_exit)
{
	struct euroc_player_config *ep_config = make_euroc_player_config(euroc_path);
	struct t_slam_tracker_config *st_config = make_slam_tracker_config(slam_config, output_path);
	// playback, not dataset: EUROC_CAM_COUNT narrows how many cameras the player actually streams
	// (a G2 dataset records all 4, SLAM runs on 2), and sizing the tracker off the dataset instead
	// makes it wait for cam2 forever -- "Assertion failed @push_frame: Expected cam2 frame,
	// received cam0" on the very first round.
	st_config->cam_count = ep_config->playback.cam_count;

	// Frame context that will manage SLAM tracker and euroc player lifetimes
	struct xrt_frame_context xfctx = {0};

	// Start SLAM tracker
	struct xrt_tracked_slam *xts = NULL;
	struct xrt_slam_sinks *sinks = NULL;
	int ret = t_slam_create(&xfctx, st_config, &xts, &sinks);
	EUROC_ASSERT(ret == 0, "Failed to create slam tracker");
	t_slam_start(xts);

	// Stream euroc player into the tracker
	struct xrt_fs *xfs = euroc_player_create(&xfctx, euroc_path, ep_config);
	xrt_fs_slam_stream_start(xfs, sinks);

	// Let's loop until both the player and the tracker finish

	// Last two tracked poses, if they are the same we assume tracking stopped
	struct xrt_space_relation a = {0};
	struct xrt_space_relation b = {0};
	b.pose.orientation.w = 42; // Make b different from a

	// The stop check still compares two samples 0.2 s apart, exactly as before; the inner loop
	// only adds the display-rate polling that makes the prediction path run (SLAM_BATCH_POLL_HZ).
	const int64_t check_period_ns = (int64_t)(0.2 * U_TIME_1S_IN_NS);
	const long poll_hz = debug_get_num_option_slam_batch_poll_hz();
	const int64_t poll_period_ns = poll_hz > 0 ? U_TIME_1S_IN_NS / poll_hz : 0;

	bool tracking = true;
	bool streaming = xrt_fs_is_running(xfs);
	while ((streaming || tracking) && !*should_exit) {
		if (poll_period_ns > 0) {
			for (int64_t waited = 0; waited < check_period_ns && !*should_exit;
			     waited += poll_period_ns) {
				struct xrt_space_relation ignored = {0};
				os_nanosleep(poll_period_ns);
				xrt_tracked_slam_get_tracked_pose(xts, os_monotonic_get_ns(), &ignored);
			}
		} else {
			os_nanosleep(check_period_ns);
		}
		a = b;
		xrt_tracked_slam_get_tracked_pose(xts, os_monotonic_get_ns(), &b);
		tracking = memcmp(&a, &b, sizeof(struct xrt_space_relation)) != 0;
		streaming = xrt_fs_is_running(xfs);
	}

	xrt_frame_context_destroy_nodes(&xfctx);
	free(st_config);
	free(ep_config);
}

#endif
