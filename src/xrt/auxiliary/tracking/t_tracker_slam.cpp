// Copyright 2021-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SLAM tracking code.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frameserver.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_var.h"
#include "util/u_trace_marker.h"

#include "os/os_threading.h"
#include <cstdio>

#include "math/m_api.h"
#include "math/m_filter_fifo.h"
#include "math/m_filter_one_euro.h"
#include "math/m_predict.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "tracking/t_euroc_recorder.h"
#include "tracking/t_openvr_tracker.h"
#include "tracking/t_tracking.h"
#include "tracking/t_vit_loader.h"
#include "tracking/t_dead_reckoning.h"

#include "vit/vit_interface.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/version.hpp>

#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <atomic>
#include <shared_mutex>
#include <string>
#include <vector>
#include <algorithm>


//! @todo Get preferred system from systems found at build time
#define PREFERRED_VIT_SYSTEM_LIBRARY "libbasalt.so"

#define SLAM_TRACE(...) U_LOG_IFL_T(t.log_level, __VA_ARGS__)
#define SLAM_DEBUG(...) U_LOG_IFL_D(t.log_level, __VA_ARGS__)
#define SLAM_INFO(...) U_LOG_IFL_I(t.log_level, __VA_ARGS__)
#define SLAM_WARN(...) U_LOG_IFL_W(t.log_level, __VA_ARGS__)
#define SLAM_ERROR(...) U_LOG_IFL_E(t.log_level, __VA_ARGS__)
#define SLAM_ASSERT(predicate, ...)                                                                                    \
	do {                                                                                                           \
		bool p = predicate;                                                                                    \
		if (!p) {                                                                                              \
			U_LOG(U_LOGGING_ERROR, __VA_ARGS__);                                                           \
			assert(false && "SLAM_ASSERT failed: " #predicate);                                            \
			exit(EXIT_FAILURE);                                                                            \
		}                                                                                                      \
	} while (false);
#define SLAM_ASSERT_(predicate) SLAM_ASSERT(predicate, "Assertion failed " #predicate)

// Debug assertions, not vital but useful for finding errors
#ifdef NDEBUG
#define SLAM_DASSERT(predicate, ...) (void)(predicate)
#define SLAM_DASSERT_(predicate) (void)(predicate)
#else
#define SLAM_DASSERT(predicate, ...) SLAM_ASSERT(predicate, __VA_ARGS__)
#define SLAM_DASSERT_(predicate) SLAM_ASSERT_(predicate)
#endif

//! @see t_slam_tracker_config
DEBUG_GET_ONCE_LOG_OPTION(slam_log, "SLAM_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_OPTION(vit_system_library_path, "VIT_SYSTEM_LIBRARY_PATH", PREFERRED_VIT_SYSTEM_LIBRARY)
DEBUG_GET_ONCE_OPTION(slam_config, "SLAM_CONFIG", nullptr)
DEBUG_GET_ONCE_BOOL_OPTION(slam_ui, "SLAM_UI", false)
DEBUG_GET_ONCE_BOOL_OPTION(slam_submit_from_start, "SLAM_SUBMIT_FROM_START", false)
DEBUG_GET_ONCE_NUM_OPTION(slam_openvr_groundtruth_device, "SLAM_OPENVR_GROUNDTRUTH_DEVICE", 0)
DEBUG_GET_ONCE_NUM_OPTION(slam_prediction_type, "SLAM_PREDICTION_TYPE", long(SLAM_PRED_DEAD_RECKONING))
DEBUG_GET_ONCE_BOOL_OPTION(slam_pred_freeze_position, "SLAM_PRED_FREEZE_POSITION", false)
DEBUG_GET_ONCE_NUM_OPTION(slam_pred_neck_arm_mm, "SLAM_PRED_NECK_ARM_MM", 0)
//! Bounded position-extrapolation horizon in ms, on top of pred_freeze_position (0 = today's
//! full freeze, unchanged default). See predict_pose()'s SLAM_PRED_POSITION_HORIZON_MS block.
DEBUG_GET_ONCE_NUM_OPTION(slam_pred_position_horizon_ms, "SLAM_PRED_POSITION_HORIZON_MS", 0)
//! Physical speed clamp (cm/s, integer env var like the others here) on the velocity the
//! horizon above extrapolates. 0 = no clamp. Default 150 (1.5 m/s, a seated head's ceiling) --
//! see predict_pose()'s SLAM_PRED_POSITION_MAX_SPEED_CM_S block for the live incident.
DEBUG_GET_ONCE_NUM_OPTION(slam_pred_position_max_speed_cm_s, "SLAM_PRED_POSITION_MAX_SPEED_CM_S", 150)
DEBUG_GET_ONCE_BOOL_OPTION(slam_write_csvs, "SLAM_WRITE_CSVS", false)
DEBUG_GET_ONCE_BOOL_OPTION(slam_features_enable, "SLAM_FEATURES_ENABLE", false)
DEBUG_GET_ONCE_BOOL_OPTION(slam_config_pipeline_only, "SLAM_CONFIG_PIPELINE_ONLY", false)
DEBUG_GET_ONCE_BOOL_OPTION(slam_auto_reset, "SLAM_AUTO_RESET", true)
DEBUG_GET_ONCE_BOOL_OPTION(slam_filter_before_predict, "SLAM_FILTER_BEFORE_PREDICT", true)
DEBUG_GET_ONCE_FLOAT_OPTION(slam_filter_pos_min_cutoff, "SLAM_FILTER_POS_MIN_CUTOFF", M_PI)
DEBUG_GET_ONCE_FLOAT_OPTION(slam_filter_rot_min_cutoff, "SLAM_FILTER_ROT_MIN_CUTOFF", M_PI)
DEBUG_GET_ONCE_FLOAT_OPTION(slam_filter_min_dcutoff, "SLAM_FILTER_MIN_DCUTOFF", 1)
DEBUG_GET_ONCE_FLOAT_OPTION(slam_filter_beta, "SLAM_FILTER_BETA", 0.16)
DEBUG_GET_ONCE_FLOAT_OPTION(slam_pos_deadzone, "SLAM_POS_DEADZONE_M", 0)
DEBUG_GET_ONCE_OPTION(slam_filter, "SLAM_FILTER", nullptr)
DEBUG_GET_ONCE_NUM_OPTION(slam_auto_reset_max_speed, "SLAM_AUTO_RESET_MAX_SPEED", 10)
DEBUG_GET_ONCE_BOOL_OPTION(slam_reset_offset_carry, "SLAM_RESET_OFFSET_CARRY", true)
DEBUG_GET_ONCE_NUM_OPTION(slam_correction_spread_ms, "SLAM_CORRECTION_SPREAD_MS", 0)
//! reverb-g2 0103: average the correction step's INPUT (the per-anchor pos/yaw delta) over the
//! last N anchors before accumulating it. 1 (default) = current behaviour.
DEBUG_GET_ONCE_NUM_OPTION(slam_correction_avg_n, "SLAM_CORRECTION_AVG_N", 1)
//! reverb-g2 0102: every N get_tracked_pose calls, log the age of the SLAM anchor the pose was
//! predicted from (when_ns - rel_ts): p50 / p90 / max over those N calls. 0 (default) = off.
DEBUG_GET_ONCE_NUM_OPTION(slam_pose_age_log, "SLAM_POSE_AGE_LOG", 0)
//! NOTE like SLAM_AUTO_RESET_MAX_SPEED above, this is DEBUG_GET_ONCE_NUM_OPTION (integers
//! only) -- the radius is in CENTIMETRES, not metres, so a real-world value (e.g. 450 for
//! 4.5 m) survives the integer parse. 0 = disabled.
DEBUG_GET_ONCE_NUM_OPTION(slam_session_anchor_radius_cm, "SLAM_SESSION_ANCHOR_RADIUS_CM", 0)
//! 0 = disabled (default), nonzero = enabled. Not a BOOL option so it matches the on/off
//! convention already used by every other divergence-guard var above.
DEBUG_GET_ONCE_NUM_OPTION(slam_quat_norm_check, "SLAM_QUAT_NORM_CHECK", 0)
DEBUG_GET_ONCE_BOOL_OPTION(euroc_record, "EUROC_RECORD", false)
DEBUG_GET_ONCE_OPTION(euroc_record_path, "EUROC_RECORD_PATH", nullptr)
DEBUG_GET_ONCE_OPTION(slam_csv_path, "SLAM_CSV_PATH", "evaluation/")
DEBUG_GET_ONCE_BOOL_OPTION(slam_timing_stat, "SLAM_TIMING_STAT", true)
DEBUG_GET_ONCE_BOOL_OPTION(slam_features_stat, "SLAM_FEATURES_STAT", true)
DEBUG_GET_ONCE_NUM_OPTION(slam_cam_count, "SLAM_CAM_COUNT", 2)

//! Namespace for the interface to the external SLAM tracking system
namespace xrt::auxiliary::tracking::slam {
constexpr int UI_TIMING_POSE_COUNT = 192;
constexpr int UI_FEATURES_POSE_COUNT = 192;
constexpr int UI_GTDIFF_POSE_COUNT = 192;

using os::Mutex;
using std::deque;
using std::ifstream;
using std::make_shared;
using std::map;
using std::ofstream;
using std::ostream;
using std::pair;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::unique_lock;
using std::vector;
using std::filesystem::create_directories;
using Trajectory = map<timepoint_ns, xrt_pose>;
using timing_sample = vector<timepoint_ns>;

using xrt::auxiliary::math::RelationHistory;


/*
 *
 * CSV Writers
 *
 */

ostream &
operator<<(ostream &os, const xrt_pose_sample &s)
{
	timepoint_ns ts = s.timestamp_ns;
	xrt_vec3 p = s.pose.position;
	xrt_quat r = s.pose.orientation;
	os << ts << ",";
	os << p.x << "," << p.y << "," << p.z << ",";
	os << r.w << "," << r.x << "," << r.y << "," << r.z << CSV_EOL;
	return os;
}

ostream &
operator<<(ostream &os, const timing_sample &timestamps)
{
	for (const timepoint_ns &ts : timestamps) {
		string delimiter = &ts != &timestamps.back() ? "," : CSV_EOL;
		os << ts << delimiter;
	}
	return os;
}

struct feature_count_sample
{
	timepoint_ns ts;
	vector<int> counts;
};

ostream &
operator<<(ostream &os, const feature_count_sample &s)
{
	os << s.ts;
	for (int count : s.counts) {
		os << "," << count;
	}
	os << CSV_EOL;
	return os;
}

//! Writes a CSV file for a particular row type
template <typename RowType> class CSVWriter
{
public:
	bool enabled; // Modified through UI

protected:
	vector<string> column_names;

private:
	string directory;
	string filename;
	ofstream file;
	bool created = false;
	Mutex mutex;

	void
	create()
	{
		create_directories(directory);
		file = ofstream{directory + "/" + filename};
		file << "#";
		for (const string &col : column_names) {
			string delimiter = &col != &column_names.back() ? "," : CSV_EOL;
			file << col << delimiter;
		}
		file << std::fixed << std::setprecision(CSV_PRECISION);
	}

public:
	CSVWriter(const string &dir, const string &fn, bool e, const vector<string> &cn = {})
	    : enabled(e), column_names(cn), directory(dir), filename(fn)
	{}

	virtual ~CSVWriter() {}

	void
	push(const RowType &row)
	{
		unique_lock lock(mutex);

		if (!enabled) {
			return;
		}

		if (!created) {
			created = true;
			create();
		}

		file << row;
	}
};

//! Writes poses and their timestamps to a CSV file
struct TrajectoryWriter : public CSVWriter<xrt_pose_sample>
{
	TrajectoryWriter(const string &dir, const string &fn, bool e) : CSVWriter<xrt_pose_sample>(dir, fn, e)
	{
		column_names = {"timestamp [ns]", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
		                "q_RS_w []",      "q_RS_x []",    "q_RS_y []",    "q_RS_z []"};
	}
};

//! Writes timestamps measured when estimating a new pose by the SLAM system
struct TimingWriter : public CSVWriter<timing_sample>
{
	TimingWriter(const string &dir, const string &fn, bool e, const vector<string> &cn)
	    : CSVWriter<timing_sample>(dir, fn, e, cn)
	{}
};

//! Writes feature information specific to a particular estimated pose
struct FeaturesWriter : public CSVWriter<feature_count_sample>
{
	FeaturesWriter(const string &dir, const string &fn, bool e, size_t cam_count)
	    : CSVWriter<feature_count_sample>(dir, fn, e)
	{
		column_names.push_back("timestamp");
		for (size_t i = 0; i < cam_count; i++) {
			column_names.push_back("cam" + to_string(i) + " feature count");
		}
	}
};

/*!
 * Main implementation of @ref xrt_tracked_slam. This is an adapter class for
 * SLAM tracking that wraps an external SLAM implementation.
 *
 * @implements xrt_tracked_slam
 * @implements xrt_frame_node
 * @implements xrt_frame_sink
 * @implements xrt_imu_sink
 * @implements xrt_pose_sink
 */
struct TrackerSlam
{
	struct xrt_tracked_slam base = {};
	struct xrt_frame_node node = {};       //!< Will be called on destruction
	struct t_vit_bundle vit;               //!< VIT system function pointers
	struct vit_tracker_extension_set exts; //!< VIT tracker supported extensions
	struct vit_tracker *tracker;           //!< Pointer to the tracker created by the loaded VIT system;

	struct xrt_slam_sinks sinks = {};                       //!< Pointers to the sinks below
	struct xrt_frame_sink cam_sinks[XRT_TRACKING_MAX_CAMS]; //!< Sends camera frames to the SLAM system
	struct xrt_imu_sink imu_sink = {};                      //!< Sends imu samples to the SLAM system
	struct xrt_pose_sink gt_sink = {};                      //!< Register groundtruth trajectory for stats
	struct xrt_hand_masks_sink hand_masks_sink = {};        //!< Register latest masks to ignore

	bool submit;        //!< Whether to submit data pushed to sinks to the SLAM tracker

	//! Teardown guard (reverb-g2 0104). xrt_frame_context_destroy_nodes runs break_apart on the
	//! newest node first, and this tracker is created after the camera source it consumes, so
	//! tracker_stop runs while the driver's USB/IMU threads can still push one more sample --
	//! and a client can still query a pose -- into a tracker Basalt has already dismantled:
	//! SIGSEGV in Tracker::pop_pose <- flush_poses <- receive_frame <- wmr_cam_usb_thread, with
	//! the main thread inside wmr_source_stream_stop (cores 2026-08-21 and twice on 2026-08-29).
	//! Every runtime call into the VIT tracker holds vit_lock shared; break_apart takes it
	//! exclusive around tracker_stop and sets `stopped`, so in-flight calls finish before the
	//! stop and later ones are dropped instead of dereferencing a stopped tracker.
	std::shared_mutex vit_lock;
	bool stopped = false;
	std::atomic<uint32_t> dropped_after_stop{0}; //!< Calls that arrived after the stop -- each was the crash
	uint32_t cam_count; //!< Number of cameras used for tracking

	struct u_var_button reset_state_btn; //!< Reset tracker state button

	enum u_logging_level log_level; //!< Logging level for the SLAM tracker, set by SLAM_LOG var

	struct xrt_slam_sinks *euroc_recorder; //!< EuRoC dataset recording sinks
	struct openvr_tracker *ovr_tracker;    //!< OpenVR lighthouse tracker

	// Used mainly for checking that the timestamps come in order
	timepoint_ns last_imu_ts;                     //!< Last received IMU sample timestamp
	vector<timepoint_ns> last_cam_ts;             //!< Last received image timestamp per cam
	bool dropping_bundle = false;                 //!< Drop the rest of a non-monotonic frame bundle

	//! Divergence guard: a VIO that loses tracking can emit a physically impossible jump and
	//! then sit there. Catch it by implied speed and restart the tracker.
	//! NOTE SLAM_AUTO_RESET_MAX_SPEED goes through DEBUG_GET_ONCE_NUM_OPTION, which parses
	//! integers only -- a fractional value silently becomes 0 and then everything "diverges".
	struct
	{
		bool enabled;
		double max_speed;        //!< m/s, above which the jump cannot be a real head motion
		timepoint_ns quiet_until; //!< Don't re-check until this timestamp (post-reset settling)
		uint64_t count;           //!< Resets performed this session
	} auto_reset;

	//! Divergence-reset frame continuity. auto_reset above restarts the external tracker,
	//! which comes back reporting poses in its OWN new coordinate frame (near its own
	//! origin) -- without this, that reads to every consumer as the position teleporting,
	//! which is exactly what shipped with patch 0023 and never got the real fix: the wearer
	//! stranded far from a healthy-metrics world with no way back short of a manual recenter
	//! (T162; recurred T202, 2026-08-17, 1502 m spike then a hard snap to [0,0,0] at
	//! t=14.1min of a real session). Keeps a persistent rigid transform from "whatever frame
	//! the tracker is using right now" to "the frame the application has been seeing since
	//! the session started", re-solved every time the tracker hands back a fresh frame (see
	//! flush_poses). SLAM_RESET_OFFSET_CARRY=0 disables this and restores the old
	//! teleport-on-reset behaviour, for an A/B.
	struct
	{
		bool enabled;
		//! Raw-tracker-frame -> output-frame transform currently in effect. Identity
		//! until the first reset of the session; re-solved (not composed) on each one,
		//! see @ref awaiting_anchor -- because @ref anchor is itself expressed in the
		//! already-corrected output frame, re-solving against it is what makes repeated
		//! resets accumulate correctly without ever multiplying stale transforms
		//! together.
		struct xrt_pose offset = XRT_POSE_IDENTITY;
		//! True from the moment a reset fires until the tracker's next pose has been
		//! used to re-solve @ref offset against @ref anchor. While true, the next pose
		//! seen by flush_poses is treated as this reset's first post-reset sample,
		//! whatever its value -- no assumption is made about it landing near the
		//! tracker's origin.
		bool awaiting_anchor = false;
		//! Last known-good OUTPUT pose (i.e. already offset-corrected), captured the
		//! instant a reset fires. Deliberately the pose from BEFORE the diverged sample
		//! that triggered the reset, not the spike itself -- the auto_reset block above
		//! detects divergence by comparing against the previous accepted pose, so that
		//! previous pose is the last one known to still be good.
		struct xrt_pose anchor = XRT_POSE_IDENTITY;
	} reset_offset;

	//! Error-feedback smoothing of the DELIVERED pose stream across SLAM anchor arrivals
	//! (SLAM_CORRECTION_SPREAD_MS, default 0/off). T202's wearer characterization: the
	//! ~200ms "readjustment" snaps ("se siente como jittering de casco") happen at 4-4.6 Hz
	//! while worn but are only weakly tied to which pose was the arriving anchor (x1.10
	//! correlation) and are rare at rest (0.088 Hz) -- so the fix has to smooth the
	//! DELIVERED stream continuously, not gate on anchor arrival the way the one euro
	//! filter already does upstream of prediction (SLAM_FILTER_BEFORE_PREDICT). This is a
	//! separate, later mechanism: every time a new anchor lands (see flush_poses), it
	//! measures how far the raw prediction basis just jumped -- comparing what dead
	//! reckoning would still be delivering from the OLD anchor at the new anchor's own
	//! timestamp against the new anchor itself -- and folds that jump into this
	//! accumulator, applied on top of every subsequently delivered pose (see
	//! apply_correction_spread) and decayed back to zero over @ref spread_s instead of
	//! landing in one frame. Position and YAW only: roll/pitch are gravity-referenced and
	//! must never lag, so they are left to jump exactly as before (see the swing-discard
	//! note in flush_poses, same trade-off 0057 already accepted for the reset-offset
	//! transform this mirrors). Bypassed entirely across a divergence auto-reset (see
	//! @ref TrackerSlam::reset_offset) -- a delta computed across two unrelated tracker
	//! frames is meaningless, not merely stale.
	struct
	{
		bool enabled = false;
		double spread_s = 0.0; //!< Decay time constant, SLAM_CORRECTION_SPREAD_MS / 1000
		xrt_vec3 pos_offset = XRT_VEC3_ZERO;
		float yaw_offset_rad = 0.0f;
		timepoint_ns last_decay_ns = 0; //!< Host monotonic clock, NOT when_ns -- see decay_correction_locked
		bool have_last_decay = false;
		Mutex mutex; //!< Same cross-thread hazard as @ref filter::mutex: compositor and the
		             //!< WMR constellation tracker both call get_tracked_pose.
		//! reverb-g2 0103 (SLAM_CORRECTION_AVG_N): ring of the last deltas; the accumulator
		//! receives their mean. Why: with the head still, each anchor's delta is the raw
		//! frame-to-frame VIO position noise (mm), and a 25-50 ms decay against a 33 ms anchor
		//! period never lets one correction settle before the next uncorrelated one lands --
		//! the spread built to hide a step instead replays the noise at camera rate (the
		//! wearer's "jitter en movimientos lentos"). Averaging the input rejects that noise at
		//! the cost of a moving average's group delay, (N-1)/2 anchor periods (N=3: one
		//! period, ~33 ms), on the correction only, not on the predicted motion.
		int avg_n = 1;
		xrt_vec3 pos_hist[8] = {};
		float yaw_hist[8] = {};
		int hist_len = 0;
		int hist_pos = 0;
	} correction;

	//! reverb-g2 0102 (SLAM_POSE_AGE_LOG): the pose age nobody measured. Under the deployed
	//! SLAM_PREDICTION_TYPE=2 the only age log in predict_pose sits in the dead-reckoning
	//! branch and never runs; this samples (when_ns - rel_ts) at every get_tracked_pose and
	//! logs p50 / p90 / max every N calls. Written under @ref correction::mutex (two callers).
	struct
	{
		int every = 0;
		int n = 0;
		float ms[1024] = {};
	} age;

	//! Session-anchor divergence guard (SLAM_SESSION_ANCHOR_RADIUS_CM, default 0/off).
	//! auto_reset above catches divergence by INSTANTANEOUS implied speed -- a single step
	//! too fast to be a real head. That guard cannot see slow accumulated drift: many small
	//! steps, each individually far below the speed threshold, that add up to the tracker
	//! reporting a position nowhere near where the session actually started. Inspired by a
	//! community fork's independent divergence-guard work (Faulto/reverb-g2-linux) which
	//! adds exactly this kind of anchor-radius check; see flush_poses for where the anchor
	//! is captured and compared, and the response path this reuses verbatim from auto_reset.
	//! NOT YET HARDWARE-VALIDATED -- reasoned addition pending a wearer A/B, default off.
	struct
	{
		bool enabled = false;
		double radius_m = 0.0;
		bool have_anchor = false;         //!< True once @ref anchor_pos holds the session's first pose
		xrt_vec3 anchor_pos = XRT_VEC3_ZERO; //!< First accepted OUTPUT-frame position this session
	} session_anchor;

	//! Orientation-quaternion sanity guard (SLAM_QUAT_NORM_CHECK, default 0/off). A
	//! corrupted pose from the VIT tracker need not show up as an impossible position or
	//! speed -- a NaN, zero, or otherwise garbage quaternion can carry a plausible-looking
	//! position while its orientation is nonsense. Checked at the very top of flush_poses'
	//! loop body, before nrot is used for anything -- in particular before reset_offset's
	//! anchor-solve, which would otherwise bake a corrupt rotation into the persistent
	//! output-frame transform for the rest of the session. Same auto_reset response path,
	//! reused verbatim. Inspired by the same community fork referenced by @ref
	//! session_anchor above. NOT YET HARDWARE-VALIDATED -- reasoned addition pending a
	//! wearer A/B, default off.
	struct
	{
		bool enabled = false;
	} quat_norm_check;

	int dropped_bundles = 0;                      //!< Consecutive bundles dropped for that reason
	//! Candidate timestamp of a rejected insane forward jump (INT64_MIN = none). A single
	//! corrupted timestamp must never become last_cam_ts[0]: accepted once, every following
	//! sane frame reads as "older" and the guard drops bundles forever (seen live 2026-08-13:
	//! one 3.6e18 ns timestamp starved SLAM silently, 11k+ bundles). A jump is only accepted
	//! as a real clock re-baseline if the NEXT cam0 frame is consistent with it.
	timepoint_ns pending_fwd_jump_ts = INT64_MIN;
	struct xrt_hand_masks_sample last_hand_masks; //!< Last received hand masks info
	Mutex last_hand_masks_mutex;                  //!< Mutex for @ref last_hand_masks

	// Prediction

	//! Type of prediction to use
	t_slam_prediction_type pred_type;
	//! When true and pred_type >= GYRO, predict ORIENTATION forward but HOLD position at the
	//! last SLAM pose instead of extrapolating linear velocity across the SLAM latency.
	//! (SLAM_PRED_FREEZE_POSITION -- rationale in predict_pose().)
	bool pred_freeze_position = false;
	//! Neck-model lever-arm length in mm (SLAM_PRED_NECK_ARM_MM, 0 = off). With
	//! pred_freeze_position, swings the frozen eye position along the neck-pivot arc as
	//! orientation is predicted forward, fixing the orientation/position timestamp split.
	long pred_neck_arm_mm = 0;
	//! Bounded position-extrapolation horizon, in seconds (SLAM_PRED_POSITION_HORIZON_MS / 1000,
	//! 0 = off). With pred_freeze_position, extrapolates REAL linear velocity for up to this
	//! many seconds instead of holding position flat for the whole anchor-age gap -- see
	//! predict_pose() for the full rationale (docs/80's "possible future refinement").
	double pred_position_horizon_s = 0.0;
	//! Physical speed clamp on the horizon's extrapolated velocity, in m/s
	//! (SLAM_PRED_POSITION_MAX_SPEED_CM_S / 100, 0 = no clamp). See predict_pose().
	double pred_position_max_speed_m_s = 0.0;
	u_var_combo pred_combo;         //!< UI combo box to select @ref pred_type
	RelationHistory slam_rels{};    //!< A history of relations produced purely from external SLAM tracker data
	int dbg_pred_every = 1;         //!< Skip X SLAM poses so that you get tracked mostly by the prediction algo
	int dbg_pred_counter = 0;       //!< SLAM pose counter for prediction debugging
	struct os_mutex lock_ff;        //!< Lock for gyro_ff and accel_ff.
	struct m_ff_vec3_f32 *gyro_ff;  //!< Last gyroscope samples
	struct m_ff_vec3_f32 *accel_ff; //!< Last accelerometer samples
	vector<u_sink_debug> ui_sink;   //!< Sink to display frames in UI of each camera

	//! Used to correct accelerometer measurements when integrating into the prediction.
	//! @todo Should be automatically computed instead of required to be filled manually through the UI.
	xrt_vec3 gravity_correction{0, 0, -MATH_GRAVITY_M_S2};

	struct xrt_space_relation last_rel = XRT_SPACE_RELATION_ZERO; //!< Last reported/tracked pose
	timepoint_ns last_ts;                                         //!< Last reported/tracked pose timestamp

	//! Filters are used to smooth out the resulting trajectory
	struct
	{
		// Moving average filter
		bool use_moving_average_filter = false;
		//! Time window in ms take the average on.
		//! Increasing it smooths out the tracking at the cost of adding delay.
		double window = 66;
		struct m_ff_vec3_f32 *pos_ff; //! Predicted positions fifo
		struct m_ff_vec3_f32 *rot_ff; //! Predicted rotations fifo (only xyz components, w is inferred)

		// Exponential smoothing filter
		bool use_exponential_smoothing_filter = false;
		float alpha = 0.1; //!< How much should we lerp towards the @p target value on each update
		struct xrt_space_relation last = XRT_SPACE_RELATION_ZERO;   //!< Last filtered relation
		struct xrt_space_relation target = XRT_SPACE_RELATION_ZERO; //!< Target relation

		/*!
		 * Guards every piece of filter state in this struct. All of these filters are stateful and
		 * were written assuming a single caller, which held until a second thread started asking
		 * this tracker for poses (the WMR constellation tracker asks for the headset's pose from
		 * its camera thread, to place head-mounted cameras in the world).
		 *
		 * Unlocked, the one euro filter computes its dt against a timestamp another thread has
		 * already moved, so dt reaches zero or goes negative, the division blows up, and the NaN is
		 * STORED in the filter -- every later pose comes back NaN with the relation still flagged
		 * valid and tracked. Measured 2026-08-12: the headset's own pose came back
		 * (-nan, nan, -nan) with flags 0x33 and XRT_SUCCESS, permanently, and only with
		 * SLAM_FILTER=one_euro.
		 */
		Mutex mutex;

		/*!
		 * Timestamp of the last pose the filters below actually consumed. They are all stateful and
		 * assume time only moves forward: the one euro filter divides by
		 * dt = when_ns - (its own last ts) (m_filter_one_euro.c, m_vec3_div_scalar) with no guard,
		 * and its ts is unsigned, so a query for an EARLIER instant underflows and the resulting
		 * NaN is stored in the filter for good.
		 *
		 * That stayed invisible while the compositor was the only caller, since it always asks
		 * about a predicted display time that advances. It stopped being invisible when the WMR
		 * constellation tracker started asking for the headset's pose at CAMERA FRAME timestamps,
		 * which are in the past: the two interleave, dt goes negative, and every pose afterwards --
		 * including the one the application sees -- comes back NaN, still flagged valid and tracked.
		 * Measured 2026-08-12; a mutex does NOT fix this, the order is the problem, not the race.
		 *
		 * Queries at or before this timestamp now skip filtering and get the raw pose, which is what
		 * a tracker wants anyway; the filters keep seeing only the forward-moving stream.
		 */
		timepoint_ns last_filtered_when_ns = INT64_MIN;

		//! Queries whose timestamp is nowhere near the real clock, see filter_pose.
		uint64_t implausible_when_count = 0;
		uint64_t applied_count = 0; //!< Poses the filters actually ran on.
		uint64_t skipped_count = 0; //!< Poses skipped for going backwards in time.
		uint64_t reused_count = 0;  //!< Of those, ones answered with @ref last_filtered.

		//! Last filtered result, handed to near-simultaneous repeat queries, see filter_pose.
		struct xrt_space_relation last_filtered = XRT_SPACE_RELATION_ZERO;
		bool have_last_filtered = false;

		// One euro filter
		bool use_one_euro_filter = false;

		/*!
		 * Run the one euro filter on the SLAM poses as they arrive, BEFORE prediction,
		 * instead of on the final predicted pose.
		 *
		 * Found 2026-08-13 (T177). The output path is
		 * `predict_pose() -> filter_pose() -> app`, so the low pass is the LAST stage and
		 * its group delay lands whole on the application with nothing downstream able to
		 * pay it back. Measured on this rig with pose-lag.py: the filtered stream the app
		 * actually receives lags raw SLAM by +42.5 ms. The wearer sees that as a ghost
		 * whose separation from the real image grows with head rotation speed (angular
		 * separation = angular velocity x lag), and it survived every parameter we tried:
		 * beta 0.16 -> 60 only shrank it, and raising the derivative cutoff 1 -> 15 did
		 * nothing, because no gain on an output-stage low pass can remove its own delay.
		 * Turning the filter off entirely (SLAM_FILTER=none) removed the ghost completely
		 * and brought back the 12 deg p99 rotational jitter it exists to suppress.
		 *
		 * With this on, the jitter is filtered where it is actually generated -- the SLAM
		 * poses -- and dead reckoning then integrates the IMU from that pose's own
		 * timestamp up to the queried time, which is exactly what cancels the group delay.
		 * Same filter, same jitter suppression, without the lag.
		 *
		 * Note it changes the filter's sample rate from the query rate (~90 Hz) to the SLAM
		 * pose rate (~30 Hz). One euro is designed for irregular sampling and uses the real
		 * dt, so this is legitimate, but it is a behaviour change and was measured, not
		 * assumed.
		 */
		bool one_euro_before_predict = false;

		//! Last RAW SLAM pose, kept so velocity and the divergence guard never compare
		//! against a filtered one. See the use site.
		struct xrt_pose last_raw = XRT_POSE_IDENTITY;
		bool have_last_raw = false;

		/*!
		 * @name Position deadband, off by default.
		 *
		 * The wearer's idea (2026-08-13, T178) after the rotational ghost was fixed and
		 * the remaining complaint was a few centimetres of position that "se reacomoda
		 * todo el tiempo" while sitting still. Framed by him as a tool to USE and to rule
		 * things out with, not as a fix -- which is the right framing, hence off by
		 * default: every offline measurement in this project reads the delivered stream,
		 * and a deadband would quietly flatten exactly the jitter we are trying to
		 * quantify.
		 *
		 * It is a deadband WITH TRACKING, not a naive threshold. A naive one holds the
		 * output frozen while the error accumulates underneath and then releases it all at
		 * once when the threshold is crossed -- trading continuous shimmer for an
		 * occasional jump, which in VR is usually worse. Here the held point is dragged
		 * along so it stays exactly `deadzone` behind the input: below the threshold
		 * nothing moves, above it the output moves continuously from where it already is.
		 * The cost is an honest, bounded position offset of at most `deadzone` in the
		 * direction of travel, never a discontinuity.
		 * @{
		 */
		float pos_deadzone_m = 0.0f;
		struct xrt_vec3 deadzone_held = XRT_VEC3_ZERO;
		bool have_deadzone_held = false;
		/*! @} */
		m_filter_euro_vec3 pos_oe;     //!< One euro position filter
		m_filter_euro_quat rot_oe;     //!< One euro rotation filter
		/*!
		 * @name One euro parameters, no longer compile-time constants.
		 *
		 * Position and orientation get their own minimum cutoff because only the
		 * ORIENTATION one was validated away from the default here: with
		 * one_euro_before_predict on, the wearer swept it live and settled on 20 Hz
		 * ("casi perfecto", a few pixels of residual shimmer), while position stayed at
		 * the original M_PI. Shipping a single shared value would have silently changed a
		 * second thing nobody looked at.
		 *
		 * Why 20 works now and did not before: at the output stage the filter WAS the
		 * continuous motion path, so loosening it put jitter straight into the image. With
		 * the reorder, dead reckoning carries continuous motion and the filter only has to
		 * keep SLAM corrections from landing hard -- a much cheaper job that tolerates far
		 * less smoothing. At 20 Hz its group delay is ~8 ms, under one 90 Hz frame, which
		 * is why the correction step stopped being visible.
		 * @{
		 */
		float pos_min_cutoff = M_PI;
		float rot_min_cutoff = M_PI;
		float min_dcutoff = 1; //!< Minimum cutoff frequency for the derivative
		float beta = 0.16;     //!< Speed coefficient
		/*! @} */

	} filter;

	// Stats and metrics

	// CSV writers for offline analysis (using pointers because of container_of)
	TimingWriter *slam_times_writer;      //!< Timestamps of the pipeline for performance analysis
	FeaturesWriter *slam_features_writer; //!< Feature tracking information for analysis
	TrajectoryWriter *slam_traj_writer;   //!< Estimated poses from the SLAM system
	TrajectoryWriter *pred_traj_writer;   //!< Predicted poses
	TrajectoryWriter *filt_traj_writer;   //!< Predicted and filtered poses

	//! Tracker timing info for performance evaluation
	struct
	{
		bool enabled = false;               //!< Whether the timing extension is enabled
		float dur_ms[UI_TIMING_POSE_COUNT]; //!< Timing durations in ms
		int idx = 0;                        //!< Index of latest entry in @p dur_ms
		u_var_combo start_ts;               //!< UI combo box to select initial timing measurement
		u_var_combo end_ts;                 //!< UI combo box to select final timing measurement
		int start_ts_idx;                   //!< Selected initial timing measurement in @p start_ts
		int end_ts_idx;                     //!< Selected final timing measurement in @p end_ts
		struct u_var_timing ui;             //!< Realtime UI for tracker durations
		vector<string> columns;             //!< Column names of the measured timestamps
		string joined_columns;              //!< Column names as a null separated string
		struct u_var_button enable_btn;     //!< Toggle tracker timing reports
	} timing;

	//! Tracker feature tracking info
	struct Features
	{
		struct FeatureCounter
		{
			//! Feature count for each frame timestamp for this camera.
			//! @note Harmless race condition over this as the UI might read this while it's being written
			deque<pair<timepoint_ns, int>> entries{};

			//! Persistently stored camera name for display in the UI
			string cam_name;

			void
			addFeatureCount(timepoint_ns ts, int count)
			{
				entries.emplace_back(ts, count);
				if (entries.size() > UI_FEATURES_POSE_COUNT) {
					entries.pop_front();
				}
			}
		};

		vector<FeatureCounter> fcs; //!< Store feature count info for each camera
		u_var_curves fcs_ui;        //!< Display of `fcs` in UI

		bool enabled = false;           //!< Whether the features extension is enabled
		struct u_var_button enable_btn; //!< Toggle extension
	} features;

	//! Ground truth related fields
	struct
	{
		Trajectory *trajectory;               //!< Empty if we've not received groundtruth
		xrt_pose origin;                      //!< First ground truth pose
		float diffs_mm[UI_GTDIFF_POSE_COUNT]; //!< Positional error wrt ground truth
		int diff_idx = 0;                     //!< Index of last error in @p diffs_mm
		struct u_var_timing diff_ui;          //!< Realtime UI for positional error
		bool override_tracking = false;       //!< Force the tracker to report gt poses instead
	} gt;
};


/*
 *
 * Timing functionality
 *
 */

//! Populates @ref TrackerSlam::timing "t.timing".columns: the fixed 2 columns every build
//! ships, plus Basalt's own per-stage breakdown if the tracker supports the pose-timing
//! extension (independent of whether that extension is actually TURNED ON -- title
//! metadata is available regardless).
//!
//! Idempotent and safe to call more than once (t_slam_create calls it once headlessly,
//! before TimingWriter's constructor runs, and timing_ui_setup below calls it again for
//! the GUI-only path -- both compute the same result).
static void
timing_columns_setup(TrackerSlam &t)
{
	// We provide two timing columns by default, even if there is no extension support
	t.timing.columns = {"sampled", "received_by_monado"};

	// Only fill the timing columns if the tracker supports pose timing
	if (t.exts.has_pose_timing) {
		vit_tracker_timing_titles titles = {};
		vit_result_t vres = t.vit.tracker_get_timing_titles(t.tracker, &titles);
		if (vres != VIT_SUCCESS) {
			SLAM_ERROR("Failed to get timing titles from tracker");
			return;
		}

		// Copies the titles locally.
		std::vector<std::string> cols(titles.titles, titles.titles + titles.count);

		t.timing.columns.insert(t.timing.columns.begin() + 1, cols.begin(), cols.end());
	}
}

static void
timing_ui_setup(TrackerSlam &t)
{
	u_var_add_ro_raw_text(&t, "\nTracker timing", "Tracker timing");

	// Setup toggle button. Its initial label reflects t.timing.enabled as it is NOW, which
	// may already be true if t_slam_create enabled the extension headlessly via
	// SLAM_TIMING_STAT -- don't reset it to false here.
	static const char *msg[2] = {"[OFF] Enable timing", "[ON] Disable timing"};
	u_var_button_cb cb = [](void *t_ptr) {
		TrackerSlam *t = (TrackerSlam *)t_ptr;
		u_var_button &btn = t->timing.enable_btn;
		bool e = !t->timing.enabled;
		snprintf(btn.label, sizeof(btn.label), "%s", msg[e]);
		std::shared_lock<std::shared_mutex> vit_guard(t->vit_lock); // 0104
		if (t->stopped) {
			return;
		}
		vit_result_t vres = t->vit.tracker_enable_extension(t->tracker, VIT_TRACKER_EXTENSION_POSE_TIMING, e);
		if (vres != VIT_SUCCESS) {
			U_LOG_IFL_E(t->log_level, "Failed to set tracker timing extension");
			return;
		}
		t->timing.enabled = e;
	};
	t.timing.enable_btn.cb = cb;
	t.timing.enable_btn.disabled = !t.exts.has_pose_timing;
	t.timing.enable_btn.ptr = &t;
	u_var_add_button(&t, &t.timing.enable_btn, msg[t.timing.enabled]);

	timing_columns_setup(t);

	// Construct null-separated array of options for the combo box
	using namespace std::string_literals;
	t.timing.joined_columns = "";
	for (const string &name : t.timing.columns) {
		t.timing.joined_columns += name + "\0"s;
	}
	t.timing.joined_columns += "\0"s;

	t.timing.start_ts.count = t.timing.columns.size();
	t.timing.start_ts.options = t.timing.joined_columns.c_str();
	t.timing.start_ts.value = &t.timing.start_ts_idx;
	t.timing.start_ts_idx = 0;
	u_var_add_combo(&t, &t.timing.start_ts, "Start timestamp");

	t.timing.end_ts.count = t.timing.columns.size();
	t.timing.end_ts.options = t.timing.joined_columns.c_str();
	t.timing.end_ts.value = &t.timing.end_ts_idx;
	t.timing.end_ts_idx = t.timing.columns.size() - 1;
	u_var_add_combo(&t, &t.timing.end_ts, "End timestamp");

	t.timing.ui.values.data = t.timing.dur_ms;
	t.timing.ui.values.length = UI_TIMING_POSE_COUNT;
	t.timing.ui.values.index_ptr = &t.timing.idx;
	t.timing.ui.reference_timing = 16.6;
	t.timing.ui.center_reference_timing = true;
	t.timing.ui.range = t.timing.ui.reference_timing;
	t.timing.ui.dynamic_rescale = true;
	t.timing.ui.unit = "ms";
	u_var_add_f32_timing(&t, &t.timing.ui, "External tracker times");
}

//! Updates timing UI with info from a computed pose and returns that info
static vector<timepoint_ns>
timing_ui_push(TrackerSlam &t, const vit_pose_t *pose, int64_t ts)
{
	timepoint_ns now = os_monotonic_get_ns();
	vector<timepoint_ns> tss = {ts, now};

	// Add extra timestamps if the SLAM tracker provides them
	if (t.timing.enabled) {
		vit_pose_timing timing;
		vit_result_t vres = t.vit.pose_get_timing(pose, &timing);
		if (vres != VIT_SUCCESS) {
			// Even if the timing is enabled, some of the poses already in the queue won't have it enabled.
			if (vres != VIT_ERROR_NOT_ENABLED) {
				SLAM_ERROR("Failed to get pose timing");
			}

			return {};
		}

		std::vector<int64_t> data(timing.timestamps, timing.timestamps + timing.count);
		tss.insert(tss.begin() + 1, data.begin(), data.end());

		// The two timestamps to compare in the graph
		timepoint_ns start = tss.at(t.timing.start_ts_idx);
		timepoint_ns end = tss.at(t.timing.end_ts_idx);

		// Push to the UI graph
		float tss_ms = (end - start) / U_TIME_1MS_IN_NS;
		t.timing.idx = (t.timing.idx + 1) % UI_TIMING_POSE_COUNT;
		t.timing.dur_ms[t.timing.idx] = tss_ms;
		constexpr float a = 1.0f / UI_TIMING_POSE_COUNT; // Exponential moving average
		t.timing.ui.reference_timing = (1 - a) * t.timing.ui.reference_timing + a * tss_ms;
	}

	return tss;
}


/*
 *
 * Feature information functionality
 *
 */

static void
features_ui_setup(TrackerSlam &t)
{
	t.features.enabled = false;

	u_var_add_ro_raw_text(&t, "\nTracker features", "Tracker features");

	// Setup toggle button
	static const char *msg[2] = {"[OFF] Enable features info", "[ON] Disable features info"};
	u_var_button_cb cb = [](void *t_ptr) {
		TrackerSlam *t = (TrackerSlam *)t_ptr;
		u_var_button &btn = t->features.enable_btn;
		bool e = !t->features.enabled;
		snprintf(btn.label, sizeof(btn.label), "%s", msg[e]);
		std::shared_lock<std::shared_mutex> vit_guard(t->vit_lock); // 0104
		if (t->stopped) {
			return;
		}
		vit_result_t vres = t->vit.tracker_enable_extension(t->tracker, VIT_TRACKER_EXTENSION_POSE_FEATURES, e);
		if (vres != VIT_SUCCESS) {
			U_LOG_IFL_E(t->log_level, "Failed to set tracker features extension");
			return;
		}
		t->features.enabled = e;
	};
	t.features.enable_btn.cb = cb;
	t.features.enable_btn.disabled = !t.exts.has_pose_features;
	t.features.enable_btn.ptr = &t;
	u_var_add_button(&t, &t.features.enable_btn, msg[t.features.enabled]);

	// Setup graph
	u_var_curve_getter getter = [](void *fs_ptr, int i) -> u_var_curve_point {
		auto *fs = (TrackerSlam::Features::FeatureCounter *)fs_ptr;
		timepoint_ns now = os_monotonic_get_ns();

		size_t size = fs->entries.size();
		if (size == 0) {
			return {0, 0};
		}

		int last_idx = size - 1;
		if (i > last_idx) {
			i = last_idx;
		}

		auto [ts, count] = fs->entries.at(last_idx - i);
		return {time_ns_to_s(now - ts), double(count)};
	};

	t.features.fcs_ui.curve_count = t.cam_count;
	t.features.fcs_ui.xlabel = "Last seconds";
	t.features.fcs_ui.ylabel = "Number of features";

	t.features.fcs.resize(t.cam_count);
	for (uint32_t i = 0; i < t.cam_count; ++i) {
		auto &fc = t.features.fcs[i];
		fc.cam_name = "Cam" + to_string(i);

		auto &fc_ui = t.features.fcs_ui.curves[i];
		fc_ui.count = UI_FEATURES_POSE_COUNT;
		fc_ui.data = &fc;
		fc_ui.getter = getter;
		fc_ui.label = fc.cam_name.c_str();
	}

	u_var_add_curves(&t, &t.features.fcs_ui, "Feature count");

	// SLAM_FEATURES_ENABLE=1 turns the per-frame feature counts on without the debug GUI, so
	// "is the visual front-end tracking anything at all?" can be answered from features.csv.
	if (debug_get_bool_option_slam_features_enable()) {
		if (!t.exts.has_pose_features) {
			SLAM_WARN("SLAM_FEATURES_ENABLE set but the tracker has no pose-features extension");
		} else {
			vit_result_t vres =
			    t.vit.tracker_enable_extension(t.tracker, VIT_TRACKER_EXTENSION_POSE_FEATURES, true);
			if (vres != VIT_SUCCESS) {
				SLAM_ERROR("Failed to enable the tracker features extension (%d)", vres);
			} else {
				t.features.enabled = true;
				snprintf(t.features.enable_btn.label, sizeof(t.features.enable_btn.label), "%s",
				         "[ON] Disable features info");
				SLAM_INFO("Feature counts enabled via SLAM_FEATURES_ENABLE");
			}
		}
	}
}

static vector<int>
features_ui_push(TrackerSlam &t, const vit_pose_t *pose, int64_t ts)
{
	if (!t.features.enabled) {
		return {};
	}

	// Push to the UI graph
	vector<int> fcs{};
	for (uint32_t i = 0; i < t.cam_count; ++i) {
		vit_pose_features features = {};
		vit_result_t vres = t.vit.pose_get_features(pose, i, &features);
		if (vres != VIT_SUCCESS) {
			// Even if the features are enabled, some of the poses already in the queue won't have it
			// enabled.
			if (vres != VIT_ERROR_NOT_ENABLED) {
				SLAM_ERROR("Failed to get pose features for camera %u", i);
			}

			return {};
		}

		t.features.fcs.at(i).addFeatureCount(ts, features.count);
		fcs.push_back(features.count);
	}

	return fcs;
}

/*
 *
 * Ground truth functionality
 *
 */

//! Gets an interpolated groundtruth pose (if available) at a specified timestamp
static xrt_pose
get_gt_pose_at(const Trajectory &gt, timepoint_ns ts)
{
	if (gt.empty()) {
		return XRT_POSE_IDENTITY;
	}

	Trajectory::const_iterator rit = gt.upper_bound(ts);

	if (rit == gt.begin()) { // Too far in the past, return first gt pose
		return gt.begin()->second;
	}

	if (rit == gt.end()) { // Too far in the future, return last gt pose
		return std::prev(gt.end())->second;
	}

	Trajectory::const_iterator lit = std::prev(rit);

	const auto &[lts, lpose] = *lit;
	const auto &[rts, rpose] = *rit;

	float t = double(ts - lts) / double(rts - lts);
	SLAM_DASSERT_(0 <= t && t <= 1);

	xrt_pose res{};
	math_quat_slerp(&lpose.orientation, &rpose.orientation, t, &res.orientation);
	res.position = m_vec3_lerp(lpose.position, rpose.position, t);
	return res;
}

//! Converts a pose from the tracker to ground truth
static struct xrt_pose
xr2gt_pose(const xrt_pose &gt_origin, const xrt_pose &xr_pose)
{
	//! @todo Right now this is hardcoded for Basalt and the EuRoC vicon datasets
	//! groundtruth and ignores orientation. Applies a fixed transformation so
	//! that the tracked and groundtruth trajectories origins and general motion
	//! match. The usual way of evaluating trajectory errors in SLAM requires to
	//! first align the trajectories through a non-linear optimization (e.g. gauss
	//! newton) so that they are as similar as possible. For this you need the
	//! entire tracked trajectory to be known beforehand, which makes it not
	//! suitable for reporting an error metric in realtime. See this 2-page paper
	//! for more info on trajectory alignment:
	//! https://ylatif.github.io/movingsensors/cameraReady/paper07.pdf

	xrt_vec3 pos = xr_pose.position;
	xrt_quat z180{0, 0, 1, 0};
	math_quat_rotate_vec3(&z180, &pos, &pos);
	math_quat_rotate_vec3(&gt_origin.orientation, &pos, &pos);
	pos += gt_origin.position;

	return {XRT_QUAT_IDENTITY, pos};
}

//! The inverse of @ref xr2gt_pose.
static struct xrt_pose
gt2xr_pose(const xrt_pose &gt_origin, const xrt_pose &gt_pose)
{
	xrt_vec3 pos = gt_pose.position;
	pos -= gt_origin.position;
	xrt_quat gt_origin_orientation_inv = gt_origin.orientation;
	math_quat_invert(&gt_origin_orientation_inv, &gt_origin_orientation_inv);
	math_quat_rotate_vec3(&gt_origin_orientation_inv, &pos, &pos);
	xrt_quat zn180{0, 0, -1, 0};
	math_quat_rotate_vec3(&zn180, &pos, &pos);

	return {XRT_QUAT_IDENTITY, pos};
}

static void
gt_ui_setup(TrackerSlam &t)
{
	u_var_add_ro_raw_text(&t, "\nTracker groundtruth", "Tracker groundtruth");
	t.gt.diff_ui.values.data = t.gt.diffs_mm;
	t.gt.diff_ui.values.length = UI_GTDIFF_POSE_COUNT;
	t.gt.diff_ui.values.index_ptr = &t.gt.diff_idx;
	t.gt.diff_ui.reference_timing = 0;
	t.gt.diff_ui.center_reference_timing = true;
	t.gt.diff_ui.range = 100; // 10cm
	t.gt.diff_ui.dynamic_rescale = true;
	t.gt.diff_ui.unit = "mm";
	u_var_add_f32_timing(&t, &t.gt.diff_ui, "Tracking absolute error");
}

static void
gt_ui_push(TrackerSlam &t, timepoint_ns ts, xrt_pose tracked_pose)
{
	if (t.gt.trajectory->empty()) {
		return;
	}

	xrt_pose gt_pose = get_gt_pose_at(*t.gt.trajectory, ts);
	xrt_pose xr_pose = xr2gt_pose(t.gt.origin, tracked_pose);

	float len_mm = m_vec3_len(xr_pose.position - gt_pose.position) * 1000;
	t.gt.diff_idx = (t.gt.diff_idx + 1) % UI_GTDIFF_POSE_COUNT;
	t.gt.diffs_mm[t.gt.diff_idx] = len_mm;
	constexpr float a = 1.0f / UI_GTDIFF_POSE_COUNT; // Exponential moving average
	t.gt.diff_ui.reference_timing = (1 - a) * t.gt.diff_ui.reference_timing + a * len_mm;
}

/*
 *
 * Tracker functionality
 *
 */

//! Forward declaration -- flush_poses needs to query "what would still be delivered from
//! the OLD anchor" for correction spreading (see below), and predict_pose is defined
//! further down in this file.
static void
predict_pose(TrackerSlam &t, timepoint_ns when_ns, struct xrt_space_relation *out_relation);

//! Advances TrackerSlam::correction's decay by however long it's been since the last
//! touch (either this call or the previous one, from either flush_poses below or
//! apply_correction_spread further down -- both callers, on possibly different threads),
//! and updates the "last touched" cursor. Must be called with @ref TrackerSlam::correction
//! "t.correction".mutex held.
//!
//! Deliberately reads the HOST monotonic clock, not when_ns: when_ns is a query timestamp
//! that can legitimately run backwards across callers (the WMR constellation tracker asks
//! about past camera frames interleaved with the compositor's forward-marching predicted
//! display time -- see TrackerSlam::filter::last_filtered_when_ns for the exact bug this
//! sidesteps: a filter that divides by when_ns-delta blew up permanently on exactly this).
//! Real elapsed wall-clock time is also the conceptually correct thing to decay against --
//! the correction represents how wrong a real dead-reckoning belief turned out to be, and
//! that staleness resolves in real time regardless of which timestamp happens to be
//! queried next.
static void
decay_correction_locked(TrackerSlam &t)
{
	timepoint_ns now_ns = (timepoint_ns)os_monotonic_get_ns();

	if (!t.correction.have_last_decay) {
		t.correction.last_decay_ns = now_ns;
		t.correction.have_last_decay = true;
		return;
	}

	double dt_s = time_ns_to_s(now_ns - t.correction.last_decay_ns);
	t.correction.last_decay_ns = now_ns;
	if (dt_s <= 0.0 || t.correction.spread_s <= 0.0) {
		return;
	}

	float decay = (float)exp(-dt_s / t.correction.spread_s);
	t.correction.pos_offset *= decay;
	t.correction.yaw_offset_rad *= decay;
}

//! Dequeue all tracked poses from the SLAM system and update prediction data with them.
static bool
flush_poses(TrackerSlam &t)
{

	vit_pose_t *pose = NULL;
	vit_result_t vres = t.vit.tracker_pop_pose(t.tracker, &pose);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to get pose from VIT tracker");
	}

	if (pose == NULL) {
		SLAM_TRACE("No poses to flush");
		return false;
	}

	do {
		// New pose
		vit_pose_data_t data;
		vres = t.vit.pose_get_data(pose, &data);
		if (vres != VIT_SUCCESS) {
			SLAM_ERROR("Failed to get pose data from VIT tracker");
			// Every other exit of this loop destroys the pose; this one leaked it
			// (found during the T204 leak hunt -- rare path, real leak).
			t.vit.pose_destroy(pose);
			return false;
		}

		int64_t nts = data.timestamp;

		xrt_vec3 npos{data.px, data.py, data.pz};
		xrt_quat nrot{data.ox, data.oy, data.oz, data.ow};
		xrt_vec3 nvel{data.vx, data.vy, data.vz};

		// Orientation-quaternion sanity guard (SLAM_QUAT_NORM_CHECK, default off -- see
		// TrackerSlam::quat_norm_check's doc comment). Must run HERE, before nrot is used
		// for anything -- in particular before the reset_offset block just below, which
		// (when awaiting_anchor) solves a persistent output-frame transform straight out of
		// nrot; a corrupt norm there wouldn't just corrupt this one pose, it would corrupt
		// every pose reset_offset carries forward for the rest of the session. Same
		// auto_reset->tracker_reset() response as the speed-based guard further down,
		// reused verbatim, including the reset_offset anchor capture so output-frame
		// continuity survives this kind of reset too -- fetched directly here rather than
		// via the `lr`/`lts` locals below, which this check runs strictly before.
		//
		// Gated on auto_reset.enabled/quiet_until too, matching every other guard in this
		// function -- SLAM_AUTO_RESET=0 must disable ALL reset-on-divergence behavior, not
		// just the speed check, and this must not be able to re-fire during the other
		// guards' post-reset settle window either (adversarial review caught both as real
		// bugs before this shipped: SLAM_AUTO_RESET=0 not fully disabling this check, and a
		// reset-loop risk on noisy re-localization poses inside quiet_until).
		if (t.quat_norm_check.enabled && t.auto_reset.enabled && nts > t.auto_reset.quiet_until) {
			double qnorm = sqrt((double)nrot.x * nrot.x + (double)nrot.y * nrot.y +
			                     (double)nrot.z * nrot.z + (double)nrot.w * nrot.w);
			// NaN-safe by construction: a comparison against NaN is always false in
			// IEEE-754, so `qnorm < 0.5 || qnorm > 1.5` would let a NaN norm (the
			// literal motivating case this guard exists for) pass through silently --
			// caught in adversarial review. Negating an inclusive "sane" range instead
			// means ANY non-finite qnorm trips the `!` and is correctly rejected.
			if (!(qnorm >= 0.5 && qnorm <= 1.5)) {
				t.auto_reset.count++;
				SLAM_WARN("Tracker diverged: orientation quaternion norm %.3f outside "
				          "[0.5, 1.5] at ts=%" PRId64 ". Resetting (reset #%" PRIu64
				          " this session).",
				          qnorm, nts, t.auto_reset.count);
				if (t.reset_offset.enabled) {
					xrt_space_relation last_good = XRT_SPACE_RELATION_ZERO;
					int64_t last_good_ts;
					t.slam_rels.get_latest(&last_good_ts, &last_good);
					// Prefer the last RAW pose over slam_rels' possibly-filtered
					// one, same preference the `lr` fetch below applies -- see
					// its comment for why (T177).
					t.reset_offset.anchor =
					    t.filter.have_last_raw ? t.filter.last_raw : last_good.pose;
					t.reset_offset.awaiting_anchor = true;
				}
				vit_result_t rres = t.vit.tracker_reset(t.tracker);
				if (rres != VIT_SUCCESS) {
					SLAM_ERROR("Failed to reset the diverged tracker (%d)", rres);
				}
				// Same post-reset settling window as the speed-based guard.
				t.auto_reset.quiet_until = nts + 2LL * U_TIME_1S_IN_NS;
				t.vit.pose_destroy(pose);
				break;
			}
		}

		// Captured BEFORE the reset_offset block below, which clears this flag the
		// instant it consumes it -- correction spreading further down needs to know
		// whether THIS pose is the reset's first post-reset anchor, and by the time it
		// runs the flag would already read false either way.
		bool was_awaiting_reset_anchor = t.reset_offset.enabled && t.reset_offset.awaiting_anchor;

		// Divergence-reset frame continuity (SLAM_RESET_OFFSET_CARRY, default on -- see
		// TrackerSlam::reset_offset's doc comment, and the auto_reset block further down
		// where a reset is detected and @ref reset_offset.awaiting_anchor gets set).
		// Applied HERE, before npos/nrot/nvel are used for ANYTHING -- the divergence
		// check just below, the relation pushed to slam_rels, prediction, every output
		// filter, and every CSV/EuRoC writer (including tracking.csv; see its own comment
		// further down for why this is a deliberate, disclosed exception to "tracking.csv
		// is unfiltered raw output" -- this is a coordinate REBASE, not smoothing, and
		// SLAM_RESET_OFFSET_CARRY=0 restores byte-identical pre-fix output) -- so every one
		// of those sees a single continuous frame and never has to know a reset happened.
		if (t.reset_offset.enabled) {
			if (t.reset_offset.awaiting_anchor) {
				// The tracker's first pose since tracker_reset() -- solve for the
				// transform T with T ∘ (this pose) landing on t.reset_offset.anchor,
				// the last good OUTPUT pose captured right before the reset.
				//
				// math_pose_transform(T, pose, out) composes as
				//   out.orientation = T.orientation * pose.orientation
				//   out.position    = T.orientation * pose.position + T.position
				// so solving out == anchor for T gives
				//   T.orientation = anchor.orientation * inverse(nrot)
				//   T.position    = anchor.position - rotate(T.orientation, npos)
				//
				// That T.orientation is the FULL relative rotation between the two
				// frames, roll/pitch included -- but Basalt re-aligns to gravity on
				// every reset, so nrot is already close to upright in its own frame,
				// and forcing the full rotation on top of that would tilt the world
				// by whatever roll/pitch noise is in the difference. This file
				// already treats +Z as up (see gravity_correction above, and that
				// its UI only ever exposes the Z component): decompose T.orientation
				// about the Z axis and keep only its TWIST (yaw), discarding the
				// SWING (roll/pitch). Re-solve the position against that yaw-only
				// rotation so the anchor still lands exactly; orientation continuity
				// is then only approximate, bounded by the roll/pitch difference
				// between the two gravity alignments, which is the intended
				// trade-off -- roll/pitch stay tracker-truth, only position and
				// heading are carried across the reset.
				xrt_quat nrot_inv;
				math_quat_invert(&nrot, &nrot_inv);
				xrt_quat full_rot;
				math_quat_rotate(&t.reset_offset.anchor.orientation, &nrot_inv, &full_rot);

				xrt_vec3 z_axis{0, 0, 1};
				xrt_quat swing_discarded, yaw_only;
				math_quat_decompose_swing_twist(&full_rot, &z_axis, &swing_discarded, &yaw_only);

				xrt_vec3 rotated_npos;
				math_quat_rotate_vec3(&yaw_only, &npos, &rotated_npos);

				t.reset_offset.offset.orientation = yaw_only;
				t.reset_offset.offset.position = t.reset_offset.anchor.position - rotated_npos;
				t.reset_offset.awaiting_anchor = false;

				float yaw_deg = 2.0f * atan2f(yaw_only.z, yaw_only.w) * (180.0f / (float)M_PI);
				xrt_vec3 d = t.reset_offset.offset.position;
				SLAM_WARN("Reset #%" PRIu64 ": carrying offset into the output pose so it "
				          "stays continuous -- position delta=[%.3f,%.3f,%.3f] m, yaw "
				          "delta=%.2f deg",
				          t.auto_reset.count, (double)d.x, (double)d.y, (double)d.z,
				          (double)yaw_deg);
			}

			xrt_pose raw_pose{nrot, npos};
			xrt_pose corrected_pose;
			math_pose_transform(&t.reset_offset.offset, &raw_pose, &corrected_pose);
			nrot = corrected_pose.orientation;
			npos = corrected_pose.position;

			// Velocity transforms by rotation only (no translation term), so rotate it
			// the same way to stay consistent with the corrected pose above for
			// whichever consumer (prediction's dead reckoning) reads it next.
			xrt_vec3 rotated_nvel;
			math_quat_rotate_vec3(&t.reset_offset.offset.orientation, &nvel, &rotated_nvel);
			nvel = rotated_nvel;
		}

		// Last relation
		xrt_space_relation lr = XRT_SPACE_RELATION_ZERO;
		int64_t lts;
		t.slam_rels.get_latest(&lts, &lr);
		xrt_quat lrot = lr.pose.orientation;

		// ...but compare RAW against RAW. With one_euro_before_predict on, the relation
		// sitting in slam_rels is a FILTERED pose, and both users of the previous pose
		// below -- the finite-difference angular velocity and the divergence guard's speed
		// check -- would then be measuring raw-minus-filtered, i.e. mostly the filter's own
		// lag. That inflates exactly when the head moves fast: the angular velocity handed
		// to prediction overshoots, and the guard sees a speed that never happened. Caught
		// 2026-08-13 (T177) right after the reorder shipped, as "tracking errors when
		// moving fast". Keeping the last raw pose costs one struct member and makes both
		// consumers independent of whatever the filter is doing.
		if (t.filter.have_last_raw) {
			lrot = t.filter.last_raw.orientation;
			lr.pose = t.filter.last_raw;
		}

		double dt = time_ns_to_s(nts - lts);

		SLAM_TRACE("Dequeued SLAM pose ts=%ld p=[%f,%f,%f] r=[%f,%f,%f,%f]", //
		           nts, data.px, data.py, data.pz, data.ox, data.oy, data.oz, data.ow);

		// Divergence guard. Measured on a Reverb G2 on 2026-08-12: a session that had been
		// holding sub-metre drift for two minutes jumped to 2459 m in one step and then
		// reported that same position, to the centimetre, for the next 90 seconds -- the
		// tracker had lost it and was not coming back, and the wearer was stranded 2.5 km
		// from where they were standing with no way to recover short of killing the service.
		// A jump like that is not a head: the threshold is in m/s and a human head peaks
		// around 2-3 m/s, so 10 m/s (36 km/h) cannot be anything but divergence.
		//
		// Restarting the tracker costs a second of re-localisation. That is a far better
		// deal than the session being over, so this defaults to on.
		//
		// dt has to clear a real floor, not just be positive. Measured live: two of the first
		// three resets fired on dt = 0.000 s, where a 5 mm difference reads as 21 m/s. Poses
		// can arrive with near-identical timestamps, and a spurious reset costs exactly the
		// tracking this guard exists to protect. A third of a 30 Hz frame is a safe floor.
		if (t.auto_reset.enabled && lts != 0 && nts > t.auto_reset.quiet_until && dt > 0.005 &&
		    dt < 0.5) {
			xrt_vec3 lpos = lr.pose.position;
			double dx = npos.x - lpos.x, dy = npos.y - lpos.y, dz = npos.z - lpos.z;
			double speed = sqrt(dx * dx + dy * dy + dz * dz) / dt;
			if (speed > t.auto_reset.max_speed) {
				t.auto_reset.count++;
				SLAM_WARN("Tracker diverged: %.3f m in %.3f s (%.2f m/s) at ts=%" PRId64
				          ". Resetting (reset #%" PRIu64 " this session).",
				          sqrt(dx * dx + dy * dy + dz * dz), dt, speed, nts,
				          t.auto_reset.count);
				if (t.reset_offset.enabled) {
					// Capture the last known-good OUTPUT pose (already
					// offset-corrected by the ingest block above) now, before
					// the reset -- deliberately lr.pose, i.e. the PREVIOUS
					// accepted pose, not the diverged sample just measured
					// above, which is the spike itself and not "good". The
					// actual transform is solved once the tracker hands back
					// its first post-reset pose, see reset_offset.awaiting_anchor
					// at the top of this loop.
					t.reset_offset.anchor = lr.pose;
					t.reset_offset.awaiting_anchor = true;
				}
				vit_result_t rres = t.vit.tracker_reset(t.tracker);
				if (rres != VIT_SUCCESS) {
					SLAM_ERROR("Failed to reset the diverged tracker (%d)", rres);
				}
				// The tracker comes back near the origin, which looks exactly like
				// another divergence. Stay quiet while it re-localises.
				t.auto_reset.quiet_until = nts + 2LL * U_TIME_1S_IN_NS;
				t.vit.pose_destroy(pose);
				break;
			}
		}

		// Session-anchor divergence guard (SLAM_SESSION_ANCHOR_RADIUS_CM, default off --
		// see TrackerSlam::session_anchor's doc comment). Catches the failure mode the
		// speed check above cannot: drift too slow, frame to frame, to ever trip a
		// speed threshold, that nonetheless carries the reported position far from where
		// the session actually started. npos here is already reset_offset-corrected (by
		// the ingest block near the top of this loop, if enabled) and has survived the
		// speed guard just above, so this is exactly "a later accepted OUTPUT-frame pose".
		//
		// The anchor is captured ONCE, from the session's first accepted pose, and never
		// recaptured on a later auto-reset -- reset_offset's whole job is keeping the
		// output frame continuous across a reset (see its doc comment), so the anchor
		// stays valid without needing to know a reset happened.
		//
		// CAVEAT, left in deliberately rather than hidden: with SLAM_RESET_OFFSET_CARRY on
		// (the default), the response below re-anchors reset_offset onto lr.pose -- the
		// last accepted OUTPUT position, i.e. wherever the drift already carried the
		// wearer to. tracker_reset() restarts the VIO's internal state, but the CARRIED
		// output position does not move back toward the session anchor by even one
		// centimetre. So a genuine slow-drift trip is not fixed by this reset the way a
		// speed-spike trip is: distance from the anchor stays at/over the radius, and
		// (rate-limited to once per quiet_until window, ~2 s) this will keep re-triggering
		// until real head motion or the tracker's own loop closure brings the position back
		// under the radius. This is a real, expected limitation of reusing the existing
		// response path unmodified, not an oversight -- flagged here for whoever runs the
		// wearer A/B this whole guard is gated behind.
		//
		// Requires reset_offset.enabled: this guard's anchor_pos is only meaningful in a
		// CONTINUOUS output frame. With SLAM_RESET_OFFSET_CARRY=0 (non-default but
		// supported), a reset from ANY guard changes the raw tracker frame with nothing
		// correcting it back, so a distance measured against an anchor captured in the old
		// frame is physically meaningless -- caught in adversarial review. Simplest correct
		// fix: this guard only runs when reset_offset is actually on.
		if (t.session_anchor.enabled && t.reset_offset.enabled && t.auto_reset.enabled &&
		    nts > t.auto_reset.quiet_until) {
			if (!t.session_anchor.have_anchor) {
				t.session_anchor.anchor_pos = npos;
				t.session_anchor.have_anchor = true;
			} else {
				xrt_vec3 apos = t.session_anchor.anchor_pos;
				double adx = npos.x - apos.x, ady = npos.y - apos.y, adz = npos.z - apos.z;
				double anchor_dist = sqrt(adx * adx + ady * ady + adz * adz);
				// NaN-safe by construction, same reasoning as the quat-norm guard
				// above: negate an inclusive "sane" range instead of comparing
				// against the failure threshold directly, so a non-finite
				// anchor_dist (e.g. from a NaN npos this guard runs AFTER the
				// quat-norm guard but not necessarily WITH it enabled) is
				// correctly treated as divergence instead of silently passing.
				if (!(anchor_dist <= t.session_anchor.radius_m)) {
					t.auto_reset.count++;
					SLAM_WARN("Tracker diverged: %.3f m from the session anchor "
					          "(radius %.3f m) at ts=%" PRId64
					          ". Resetting (reset #%" PRIu64 " this session).",
					          anchor_dist, t.session_anchor.radius_m, nts,
					          t.auto_reset.count);
					if (t.reset_offset.enabled) {
						// Same "last known-good OUTPUT pose" capture as the
						// speed guard above -- see its comment.
						t.reset_offset.anchor = lr.pose;
						t.reset_offset.awaiting_anchor = true;
					}
					vit_result_t rres = t.vit.tracker_reset(t.tracker);
					if (rres != VIT_SUCCESS) {
						SLAM_ERROR("Failed to reset the diverged tracker (%d)",
						           rres);
					}
					t.auto_reset.quiet_until = nts + 2LL * U_TIME_1S_IN_NS;
					t.vit.pose_destroy(pose);
					break;
				}
			}
		}

		// Compute new relation based on new pose and velocities since last pose
		xrt_space_relation rel{};
		rel.relation_flags = XRT_SPACE_RELATION_BITMASK_ALL;
		rel.pose = {nrot, npos};
		rel.linear_velocity = nvel;
		math_quat_finite_difference(&lrot, &nrot, dt, &rel.angular_velocity);

		t.filter.last_raw = {nrot, npos};
		t.filter.have_last_raw = true;

		// Filter HERE, on the SLAM pose, so that prediction still gets to run afterwards
		// and pay back the filter's group delay (see one_euro_before_predict). The
		// angular/linear velocities above are deliberately left computed from the RAW
		// poses: they feed prediction, and smoothing them would put the lag straight back
		// in through the other door.
		if (t.filter.use_one_euro_filter && t.filter.one_euro_before_predict) {
			m_filter_euro_vec3_run(&t.filter.pos_oe, nts, &rel.pose.position, &rel.pose.position);
			m_filter_euro_quat_run(&t.filter.rot_oe, nts, &rel.pose.orientation,
			                       &rel.pose.orientation);
		}

		// Push to relationship history unless we are debugging prediction
		if (t.dbg_pred_counter % t.dbg_pred_every == 0) {
			// Error-feedback correction spreading (SLAM_CORRECTION_SPREAD_MS -- see
			// TrackerSlam::correction's doc comment). Has to run against the OLD,
			// pre-push slam_rels/IMU state, so it captures exactly the jump THIS
			// anchor is about to introduce -- hence strictly before the push below,
			// not after.
			if (t.correction.enabled) {
				if (was_awaiting_reset_anchor) {
					// This pose is the reset's own first post-reset anchor --
					// the world just got rebased by the block above, and any
					// correction accumulated against the OLD frame no longer
					// means anything. Drop it rather than fold a delta computed
					// across two unrelated tracker frames into the accumulator.
					unique_lock corr_lock(t.correction.mutex);
					t.correction.pos_offset = XRT_VEC3_ZERO;
					t.correction.yaw_offset_rad = 0.0f;
					t.correction.hist_len = 0; // 0103: deltas across a reset mean nothing
					t.correction.hist_pos = 0;
				} else {
					// "What would still be delivered from the OLD anchor, dead
					// reckoned up to THIS anchor's own timestamp" -- i.e. exactly
					// predict_pose's normal job, called one query early, before
					// slam_rels sees the new pose.
					xrt_space_relation prev_rel{};
					predict_pose(t, nts, &prev_rel);
					bool prev_valid =
					    (prev_rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) &&
					    (prev_rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
					if (prev_valid) {
						xrt_vec3 pos_delta = prev_rel.pose.position - rel.pose.position;

						// Yaw-only relative rotation from the new anchor to
						// what was still being delivered -- same swing-discard
						// trade-off as the reset-offset transform above: roll/
						// pitch are gravity-truth and must jump instantly, only
						// heading gets spread.
						xrt_quat new_rot_inv;
						math_quat_invert(&rel.pose.orientation, &new_rot_inv);
						xrt_quat full_delta;
						math_quat_rotate(&prev_rel.pose.orientation, &new_rot_inv,
						                 &full_delta);
						xrt_vec2 swing_discarded;
						float yaw_delta_rad;
						math_quat_to_swing_twist(&full_delta, &swing_discarded,
						                          &yaw_delta_rad);

						unique_lock corr_lock(t.correction.mutex);
						decay_correction_locked(t);
						if (t.correction.avg_n > 1) {
							// 0103: accumulate the mean of the last avg_n deltas
							auto &c = t.correction;
							c.pos_hist[c.hist_pos] = pos_delta;
							c.yaw_hist[c.hist_pos] = yaw_delta_rad;
							c.hist_pos = (c.hist_pos + 1) % c.avg_n;
							if (c.hist_len < c.avg_n) {
								c.hist_len++;
							}
							xrt_vec3 pos_mean = XRT_VEC3_ZERO;
							float yaw_mean = 0.0f;
							for (int i = 0; i < c.hist_len; i++) {
								pos_mean += c.pos_hist[i];
								yaw_mean += c.yaw_hist[i];
							}
							pos_delta = pos_mean * (1.0f / (float)c.hist_len);
							yaw_delta_rad = yaw_mean / (float)c.hist_len;
						}
						t.correction.pos_offset += pos_delta;
						t.correction.yaw_offset_rad += yaw_delta_rad;
					}
				}
			}

			t.slam_rels.push(rel, nts);
		}
		t.dbg_pred_counter = (t.dbg_pred_counter + 1) % t.dbg_pred_every;

		// tracking.csv must keep meaning "raw SLAM output" -- it is the baseline every
		// offline tool in this project compares against (docs/32, pose-lag.py). Filtering
		// in the ingest path above silently redefined it, which would have made the next
		// lag measurement compare the filter against itself and read a clean zero. Same
		// for the EuRoC recorder: a dataset recorded through our filter is not a dataset.
		// The one deliberate exception, disclosed here rather than silently changing what
		// "raw" means again: when SLAM_RESET_OFFSET_CARRY is on (default), npos/nrot were
		// already rebased by TrackerSlam::reset_offset above, before this point -- so after
		// an auto-reset, tracking.csv keeps recording one continuous trajectory instead of
		// jumping to the tracker's own post-reset origin. That's a coordinate REBASE, not
		// smoothing: it doesn't add noise or remove signal the way the filters would, and
		// SLAM_RESET_OFFSET_CARRY=0 restores byte-identical pre-fix raw output (including
		// the teleport) for anyone who needs the tracker's literal, un-rebased numbers.
		xrt_pose raw_pose = {nrot, npos};
		gt_ui_push(t, nts, raw_pose);
		t.slam_traj_writer->push({nts, raw_pose});
		xrt_pose_sample pose_sample = {nts, raw_pose};
		xrt_sink_push_pose(t.euroc_recorder->gt, &pose_sample);

		auto tss = timing_ui_push(t, pose, nts);
		t.slam_times_writer->push(tss);

		if (t.features.enabled) {
			vector feat_count = features_ui_push(t, pose, nts);
			t.slam_features_writer->push({nts, feat_count});
		}

		t.vit.pose_destroy(pose);
	} while (t.vit.tracker_pop_pose(t.tracker, &pose) == VIT_SUCCESS && pose);

	return true;
}

//! Return our best guess of the relation at time @p when_ns using all the data the tracker has.
static void
predict_pose(TrackerSlam &t, timepoint_ns when_ns, struct xrt_space_relation *out_relation)
{
	XRT_TRACE_MARKER();

	bool valid_pred_type = t.pred_type >= SLAM_PRED_NONE && t.pred_type < SLAM_PRED_COUNT;
	SLAM_DASSERT(valid_pred_type, "Invalid prediction type (%d)", t.pred_type);

	// Get last relation computed purely from SLAM data
	xrt_space_relation rel{};
	int64_t rel_ts;
	bool empty = !t.slam_rels.get_latest(&rel_ts, &rel);

	// Stop if there is no previous relation to use for prediction
	if (empty) {
		out_relation->relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
		return;
	}

	// Use only last SLAM pose without prediction if PREDICTION_NONE
	if (t.pred_type == SLAM_PRED_NONE) {
		*out_relation = rel;
		return;
	}

	// Use only SLAM data if asking for an old point in time or PREDICTION_SP_SO_SA_SL
	SLAM_DASSERT_(rel_ts < INT64_MAX);
	if (t.pred_type == SLAM_PRED_POSE_ONLY || when_ns <= (int64_t)rel_ts) {
		t.slam_rels.get(when_ns, out_relation);
		return;
	}


	if (t.pred_type == SLAM_PRED_DEAD_RECKONING) {
		// Diagnostic for the 2026-08-17 constant ~1 s wearer lag: the integrator never
		// fails (zero SLAM_ERROR hits) yet the delivered stream matches the filtered
		// anchor exactly, so either the anchor's stamp is near-now (timestamp-domain
		// skew: nothing to integrate, staleness unbridgeable) or the gap is real and
		// integration is inert some other way. Log the one number that discriminates:
		// the anchor age the integrator actually sees. Rate-limited, ~1 line/s at 250 Hz.
		static int aage_counter = 0;
		if ((aage_counter++ % 256) == 0) {
			SLAM_INFO("pred: anchor age %.1f ms (when_ns=%" PRId64 " rel_ts=%" PRId64 ")",
			          (double)(when_ns - (int64_t)rel_ts) / 1e6, when_ns, (int64_t)rel_ts);
		}

		os_mutex_lock(&t.lock_ff);

		if (!t_apply_dead_reckoning(   //
		        t.gyro_ff,             //
		        t.accel_ff,            //
		        &t.gravity_correction, //
		        when_ns,               //
		        &rel,                  //
		        (int64_t)rel_ts,       //
		        out_relation)) {
			SLAM_ERROR("Failed to apply dead reckoning prediction. Did we stop getting SLAM poses?");
			*out_relation = rel;
		}

		os_mutex_unlock(&t.lock_ff);
		return;
	}

	os_mutex_lock(&t.lock_ff);

	// Update angular velocity with gyro data
	if (t.pred_type >= SLAM_PRED_GYRO) {
		xrt_vec3 avg_gyro{};
		m_ff_vec3_f32_filter(t.gyro_ff, rel_ts, when_ns, &avg_gyro);
		math_quat_rotate_derivative(&rel.pose.orientation, &avg_gyro, &rel.angular_velocity);
	}

	// Update linear velocity with accel data
	if (t.pred_type >= SLAM_PRED_ACCEL_GYRO) {
		xrt_vec3 avg_accel{};
		m_ff_vec3_f32_filter(t.accel_ff, rel_ts, when_ns, &avg_accel);
		xrt_vec3 world_accel{};
		math_quat_rotate_vec3(&rel.pose.orientation, &avg_accel, &world_accel);
		world_accel += t.gravity_correction;
		double slam_to_imu_dt = time_ns_to_s(t.last_imu_ts - rel_ts);
		rel.linear_velocity += world_accel * slam_to_imu_dt;
	}

	os_mutex_unlock(&t.lock_ff);

	// SLAM_PRED_FREEZE_POSITION: predict orientation forward (gyro/accel above) but HOLD
	// position at the last SLAM pose instead of extrapolating it. Rationale (2026-08-26,
	// wearer A/B on Aircar 6dof, seated): during a fast head yaw the head's optical centre
	// traces an arc about the neck, so SLAM reports a real linear velocity; do_position()
	// (m_predict.c) then extrapolates that velocity across the FULL ~120-190 ms SLAM latency
	// (slam_to_now_dt below), overshooting the seat by ~50 cm and accumulating with continued
	// rotation. Orientation prediction over that window is accurate (the gyro is clean);
	// position prediction over it is not. Clearing the linear-velocity valid bit makes
	// do_position() hold pose.position while do_orientation() still applies angular_velocity,
	// giving SLAM_PRED_NONE's rock-solid position with SLAM_PRED_GYRO's head-turn
	// responsiveness. The lost real arc translation is a few cm; the removed drift is ~50 cm.
	// Saved BEFORE freezing, for SLAM_PRED_POSITION_HORIZON_MS below -- rel.linear_velocity
	// still holds Basalt's own raw SLAM-reported velocity at this point (SLAM_PRED_GYRO does
	// not touch it; only SLAM_PRED_ACCEL_GYRO's block above does, and that ran already if
	// applicable), which is exactly the real signal a bounded extrapolation needs.
	xrt_vec3 pre_freeze_linear_velocity = rel.linear_velocity;

	if (t.pred_freeze_position) {
		rel.linear_velocity = xrt_vec3{0, 0, 0};
		rel.relation_flags = (enum xrt_space_relation_flags)(rel.relation_flags &
		                                                     ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
	}

	// Do the prediction based on the updated relation
	double slam_to_now_dt = time_ns_to_s(when_ns - rel_ts);
	xrt_space_relation predicted_relation{};
	m_predict_relation(&rel, slam_to_now_dt, &predicted_relation);

	// SLAM_PRED_POSITION_HORIZON_MS (default 0, off -- unchanged behavior): a bounded middle
	// ground between full FREEZE (0 translation over the whole ~120-190ms anchor-age gap,
	// today's default) and no freeze at all (velocity*dt over the FULL gap, the ~50cm overshoot
	// FREEZE was built to remove). Extrapolates the real SLAM-reported linear velocity, but only
	// for up to this many seconds; the remainder of the gap (if slam_to_now_dt exceeds the
	// horizon) stays flat, same as full freeze. Computed manually here rather than by passing a
	// shorter delta_s into m_predict_relation() above, because that call's ONE delta_s also
	// drives do_orientation() -- orientation must still predict over the FULL slam_to_now_dt
	// (the gyro is clean over that whole window, per FREEZE_POSITION's own rationale above), only
	// position gets the shorter horizon. Stacks additively with SLAM_PRED_NECK_ARM_MM below (one
	// compensates real head TRANSLATION during the gap, the other compensates the neck-pivot
	// ROTATIONAL arc) -- independent corrections to the same position field, no interaction.
	// docs/80's own "possible future refinement (untested)" note named this exact idea.
	//
	// SLAM_PRED_POSITION_MAX_SPEED_CM_S: physical speed clamp on that extrapolated velocity.
	// Found live 2026-08-27 on the first wearer test of this horizon (50ms): "1-2-3 metros
	// fuera de la cabina" after a few fast turns. The session's own tracking.csv showed why --
	// raw SLAM anchor-to-anchor speed is p50 0.04 m/s, p99 1.66 m/s (plausible), but
	// p99.9 = 81 m/s and max = 127 m/s, i.e. ~0.2% of anchors are re-localization jumps, not
	// motion. 127 m/s x 50 ms = 6.4 m in ONE frame. Full FREEZE (0097) never saw this because
	// zeroing the velocity also silently discarded the spikes; this horizon let them through.
	// A seated head does not exceed ~1-1.5 m/s, so a clamp at that level passes essentially
	// all real motion (p99 already sits at the boundary) and kills the tails. Clamps the
	// MAGNITUDE (direction preserved), so a real fast motion still points the right way, just
	// bounded. 0 = no clamp (the original, tail-vulnerable behavior, kept only for A/B).
	if (t.pred_freeze_position && t.pred_position_horizon_s > 0.0) {
		double capped_dt = slam_to_now_dt < t.pred_position_horizon_s ? slam_to_now_dt
		                                                              : t.pred_position_horizon_s;
		xrt_vec3 v = pre_freeze_linear_velocity;
		if (t.pred_position_max_speed_m_s > 0.0) {
			double speed = sqrt((double)v.x * v.x + (double)v.y * v.y + (double)v.z * v.z);
			// NaN-safe negated form (same reasoning as the 0099 guards in this file): a
			// non-finite speed must clamp to zero, not slip through a `speed > max` test.
			if (!(speed <= t.pred_position_max_speed_m_s)) {
				float s = (speed > 0.0 && std::isfinite(speed))
				              ? (float)(t.pred_position_max_speed_m_s / speed)
				              : 0.0f;
				v.x *= s;
				v.y *= s;
				v.z *= s;
			}
		}
		predicted_relation.pose.position.x += v.x * (float)capped_dt;
		predicted_relation.pose.position.y += v.y * (float)capped_dt;
		predicted_relation.pose.position.z += v.z * (float)capped_dt;
	}

	// SLAM_PRED_NECK_ARM_MM: neck-model kinematic compensation for the orientation/position
	// timestamp split (2026-08-26, docs/80). do_orientation() above predicted orientation
	// forward to display time while position stayed at the (stale) anchor -- freezing removed
	// the linear-velocity overshoot but left orientation "now" paired with position "~150 ms
	// ago", read by the wearer as the seat sliding on fast turns. A real seated head rotates
	// about a neck pivot BEHIND+BELOW the eyes, so the eye should swing along that arc as
	// orientation advances. Model the eye's offset from the pivot as `arm` (head-local); the
	// pivot is ~fixed in world during a head turn, so the eye's world position at the PREDICTED
	// orientation is anchor_pos + (R_pred - R_anchor)*arm. Only meaningful when position is
	// frozen (otherwise it stacks on the velocity extrapolation). Length swept live via the env
	// (mm); direction is a forward-dominant up+forward eye offset (OpenXR -Z fwd/+Y up) so a
	// yaw about the neck swings the eye horizontally -- the dominant failing axis.
	if (t.pred_freeze_position && t.pred_neck_arm_mm != 0) {
		float len_m = (float)t.pred_neck_arm_mm / 1000.0f;
		xrt_vec3 arm{0.0f * len_m, 0.6f * len_m, -0.8f * len_m};  // unit dir (0,0.6,-0.8)
		xrt_vec3 arm_anchor{}, arm_pred{};
		math_quat_rotate_vec3(&rel.pose.orientation, &arm, &arm_anchor);
		math_quat_rotate_vec3(&predicted_relation.pose.orientation, &arm, &arm_pred);
		predicted_relation.pose.position.x += arm_pred.x - arm_anchor.x;
		predicted_relation.pose.position.y += arm_pred.y - arm_anchor.y;
		predicted_relation.pose.position.z += arm_pred.z - arm_anchor.z;
	}

	*out_relation = predicted_relation;
}

//! Adds TrackerSlam::correction's decaying position/yaw offset on top of a freshly
//! predicted pose (SLAM_CORRECTION_SPREAD_MS -- see that struct's doc comment). Called
//! between predict_pose and filter_pose in t_slam_get_tracked_pose: after predict_pose so
//! pred_traj_writer/pred_traj.csv keeps meaning "raw prediction, no correction spreading"
//! for anyone diffing against it (same discipline this file already applies to
//! tracking.csv), before filter_pose so the correction is included in what
//! filt_traj_writer/filt_traj.csv and the application both actually receive -- and so a
//! post-predict one euro filter (SLAM_FILTER_BEFORE_PREDICT=0) gets a chance to smooth it
//! rather than the two mechanisms operating on disjoint pieces of the stream.
static void
apply_correction_spread(TrackerSlam &t, struct xrt_space_relation *out_relation)
{
	if (!t.correction.enabled) {
		return;
	}

	// Nothing to add on top of a relation that isn't actually tracked.
	if (!(out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) ||
	    !(out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) {
		return;
	}

	unique_lock corr_lock(t.correction.mutex);
	decay_correction_locked(t);

	out_relation->pose.position += t.correction.pos_offset;

	if (t.correction.yaw_offset_rad != 0.0f) {
		xrt_vec2 zero_swing{0, 0};
		xrt_quat yaw_quat;
		math_quat_from_swing_twist(&zero_swing, t.correction.yaw_offset_rad, &yaw_quat);
		xrt_quat corrected;
		math_quat_rotate(&yaw_quat, &out_relation->pose.orientation, &corrected);
		out_relation->pose.orientation = corrected;
	}
}

//! Various filters to remove noise from the predicted trajectory.
static void
filter_pose(TrackerSlam &t, timepoint_ns when_ns, struct xrt_space_relation *out_relation)
{
	XRT_TRACE_MARKER();

	if (t.filter.use_moving_average_filter) {
		if (out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) {
			xrt_vec3 pos = out_relation->pose.position;
			m_ff_vec3_f32_push(t.filter.pos_ff, &pos, when_ns);
		}

		if (out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) {
			// Don't save w component as we can retrieve it knowing these are (almost) unit quaternions
			xrt_vec3 rot = {out_relation->pose.orientation.x, out_relation->pose.orientation.y,
			                out_relation->pose.orientation.z};
			m_ff_vec3_f32_push(t.filter.rot_ff, &rot, when_ns);
		}

		// Get averages in time window
		timepoint_ns window = t.filter.window * U_TIME_1MS_IN_NS;
		xrt_vec3 avg_pos;
		m_ff_vec3_f32_filter(t.filter.pos_ff, when_ns - window, when_ns, &avg_pos);
		xrt_vec3 avg_rot; // Naive but good enough rotation average
		m_ff_vec3_f32_filter(t.filter.rot_ff, when_ns - window, when_ns, &avg_rot);

		// Considering the naive averaging this W is a bit wrong, but it feels reasonably well
		float avg_rot_w = sqrtf(1 - (avg_rot.x * avg_rot.x + avg_rot.y * avg_rot.y + avg_rot.z * avg_rot.z));
		out_relation->pose.orientation = xrt_quat{avg_rot.x, avg_rot.y, avg_rot.z, avg_rot_w};
		out_relation->pose.position = avg_pos;

		//! @todo Implement the quaternion averaging with a m_ff_vec4_f32 and
		//! normalization. Although it would be best to have a way of generalizing
		//! types before so as to not have redundant copies of ff logic.
	}

	// See TrackerSlam::filter::mutex and last_filtered_when_ns -- every filter below mutates state
	// shared across callers, and none of them tolerate time going backwards.
	unique_lock filter_lock(t.filter.mutex);

	// Sanity-check against the real clock FIRST, not against the previous query. Measured
	// 2026-08-12: a caller passed when_ns = 5216735132270448003 (~165 years, against normal values
	// around 7e12) exactly once, and a plain "must be newer than the last one" latch then rejected
	// every subsequent pose for the entire session -- 11 filtered against 3489 skipped, i.e. the one
	// euro filter silently switched itself off and the wearer felt the raw jitter. An absolute bound
	// cannot be poisoned by a single bad value the way a latch can.
	const timepoint_ns now_ns = (timepoint_ns)os_monotonic_get_ns();
	const timepoint_ns FILTER_PLAUSIBLE_NS = 5LL * U_TIME_1S_IN_NS;
	if (when_ns < now_ns - FILTER_PLAUSIBLE_NS || when_ns > now_ns + FILTER_PLAUSIBLE_NS) {
		t.filter.implausible_when_count++;
		if (t.filter.implausible_when_count == 1 || (t.filter.implausible_when_count % 1000) == 0) {
			SLAM_WARN("filter_pose: implausible when_ns %lld (now %lld), pose returned unfiltered "
			          "[%lu so far]",
			          (long long)when_ns, (long long)now_ns,
			          (unsigned long)t.filter.implausible_when_count);
		}
		return; // out_relation stays raw, and the latch below is deliberately NOT moved
	}

	// Among plausible timestamps, the filters still need time to move forward -- see
	// last_filtered_when_ns.
	if (when_ns <= t.filter.last_filtered_when_ns) {
		t.filter.skipped_count++;
		// A query for essentially the same instant as the last filtered one -- measured here as
		// pairs 17-19 us apart, the second always slightly earlier -- must not come back RAW while
		// its twin came back filtered, or two views of the same frame disagree by the whole
		// amplitude of the jitter the filter exists to remove. Hand back the last filtered result
		// instead; over tens of microseconds the difference is nothing.
		//
		// Genuinely older queries (the constellation tracker asks about camera frames ~30 ms back)
		// fall through and get the raw pose, which is the honest answer for a different instant and
		// what a tracker wants anyway.
		const timepoint_ns FILTER_SAME_INSTANT_NS = 2LL * U_TIME_1MS_IN_NS;
		if (t.filter.have_last_filtered &&
		    when_ns > t.filter.last_filtered_when_ns - FILTER_SAME_INSTANT_NS) {
			*out_relation = t.filter.last_filtered;
			t.filter.reused_count++;
		}
		return;
	}
	t.filter.last_filtered_when_ns = when_ns;

	// Cheap standing answer to "is the filter actually doing anything". It silently stopped once
	// already (see last_filtered_when_ns) and nothing would have shown it; a ratio in the log does.
	t.filter.applied_count++;
	if ((t.filter.applied_count % 5000) == 0) {
		SLAM_INFO("filter_pose: applied %lu, skipped %lu (%lu answered from the last filtered "
		          "pose), %lu implausible",
		          (unsigned long)t.filter.applied_count, (unsigned long)t.filter.skipped_count,
		          (unsigned long)t.filter.reused_count,
		          (unsigned long)t.filter.implausible_when_count);
	}

	if (t.filter.use_exponential_smoothing_filter) {
		xrt_space_relation &last = t.filter.last;
		xrt_space_relation &target = t.filter.target;
		target = *out_relation;
		m_space_relation_interpolate(&last, &target, t.filter.alpha, target.relation_flags, &last);
		*out_relation = last;
	}

	// Skipped when the filter already ran on the incoming SLAM poses -- running it twice
	// would stack two group delays, which is the exact thing this is trying to remove.
	if (t.filter.use_one_euro_filter && !t.filter.one_euro_before_predict) {
		xrt_pose &p = out_relation->pose;
		if (out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) {
			m_filter_euro_vec3_run(&t.filter.pos_oe, when_ns, &p.position, &p.position);
		}
		if (out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) {
			m_filter_euro_quat_run(&t.filter.rot_oe, when_ns, &p.orientation, &p.orientation);
		}
	}

	// Position deadband with tracking, at the OUTPUT stage -- and it has to be here, unlike
	// the one euro reorder above. Tried first at the SLAM ingest by analogy with the
	// rotation fix, and it did nothing at all: with SLAM_PRED_DEAD_RECKONING the position
	// handed out is the SLAM position PLUS accelerometer integration since that pose, so
	// freezing the input position leaves the prediction free to move the output anyway.
	// The wearer proved it in one try by setting the threshold to 30 metres and seeing no
	// change whatsoever. The asymmetry is real and worth keeping in mind: ROTATION gains
	// from being filtered before prediction, because gyro integration faithfully restores
	// what the filter delayed; POSITION does not, because accelerometer double integration
	// re-injects noise instead of restoring signal.
	if (t.filter.pos_deadzone_m > 0.0f &&
	    (out_relation->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT)) {
		xrt_vec3 &p = out_relation->pose.position;
		if (!t.filter.have_deadzone_held) {
			t.filter.deadzone_held = p;
			t.filter.have_deadzone_held = true;
		}
		xrt_vec3 h = t.filter.deadzone_held;
		double dx = p.x - h.x, dy = p.y - h.y, dz = p.z - h.z;
		double dist = sqrt(dx * dx + dy * dy + dz * dz);
		double dzm = t.filter.pos_deadzone_m;
		if (dist > dzm && dist > 1e-9) {
			// Drag the held point along so it trails the input by exactly the
			// threshold: continuous motion, never a release-all-at-once jump.
			double k = (dist - dzm) / dist;
			h.x += (float)(dx * k);
			h.y += (float)(dy * k);
			h.z += (float)(dz * k);
			t.filter.deadzone_held = h;
		}
		p = t.filter.deadzone_held;
	}

	t.filter.last_filtered = *out_relation;
	t.filter.have_last_filtered = true;
}

static void
setup_ui(TrackerSlam &t)
{
	t.pred_combo.count = SLAM_PRED_COUNT;
	t.pred_combo.options = "NONE\0POSE_ONLY\0GYRO\0ACCEL_GYRO\0DEAD_RECKONING\0";
	t.pred_combo.value = (int *)&t.pred_type;
	t.ui_sink = vector<u_sink_debug>(t.cam_count);
	for (size_t i = 0; i < t.ui_sink.size(); i++) {
		u_sink_debug_init(&t.ui_sink[i]);
	}
	os_mutex_init(&t.lock_ff);
	m_ff_vec3_f32_alloc(&t.gyro_ff, 1000);
	m_ff_vec3_f32_alloc(&t.accel_ff, 1000);
	m_ff_vec3_f32_alloc(&t.filter.pos_ff, 1000);
	m_ff_vec3_f32_alloc(&t.filter.rot_ff, 1000);

	u_var_add_root(&t, "SLAM Tracker", true);
	u_var_add_log_level(&t, &t.log_level, "Log Level");
	u_var_add_bool(&t, &t.submit, "Submit data to SLAM");

	u_var_button_cb reset_state_cb = [](void *t_ptr) {
		TrackerSlam &t = *(TrackerSlam *)t_ptr;

		std::shared_lock<std::shared_mutex> vit_guard(t.vit_lock); // 0104
		if (t.stopped) {
			return;
		}
		vit_result_t vres = t.vit.tracker_reset(t.tracker);
		if (vres != VIT_SUCCESS) {
			SLAM_WARN("Failed to reset VIT tracker");
		}
	};
	t.reset_state_btn.cb = reset_state_cb;
	t.reset_state_btn.ptr = &t;
	u_var_add_button(&t, &t.reset_state_btn, "Reset tracker state");

	u_var_add_bool(&t, &t.gt.override_tracking, "Track with ground truth (if available)");
	euroc_recorder_add_ui(t.euroc_recorder, &t, "");

	u_var_add_gui_header(&t, NULL, "Trajectory Filter");
	u_var_add_bool(&t, &t.filter.use_moving_average_filter, "Enable moving average filter");
	u_var_add_f64(&t, &t.filter.window, "Window size (ms)");
	u_var_add_bool(&t, &t.filter.use_exponential_smoothing_filter, "Enable exponential smoothing filter");
	u_var_add_f32(&t, &t.filter.alpha, "Smoothing factor");
	u_var_add_bool(&t, &t.filter.use_one_euro_filter, "Enable one euro filter");
	// Live-toggleable on purpose: it is the A/B this was written for, and a wearer can
	// flip it mid-session and say which one ghosts, which beats any offline metric here.
	u_var_add_bool(&t, &t.filter.one_euro_before_predict, "One euro BEFORE prediction");
	// Sweepable live, which is the whole point: the usable threshold is a comfort
	// judgement only the wearer can make, and 0 (off) must stay one drag away.
	u_var_add_f32(&t, &t.filter.pos_deadzone_m, "Position deadband (m, 0 = off)");
	u_var_add_f32(&t, &t.filter.pos_oe.base.fc_min, "Position minimum cutoff");
	u_var_add_f32(&t, &t.filter.pos_oe.base.beta, "Position beta speed");
	u_var_add_f32(&t, &t.filter.pos_oe.base.fc_min_d, "Position minimum delta cutoff");
	u_var_add_f32(&t, &t.filter.rot_oe.base.fc_min, "Orientation minimum cutoff");
	u_var_add_f32(&t, &t.filter.rot_oe.base.beta, "Orientation beta speed");
	u_var_add_f32(&t, &t.filter.rot_oe.base.fc_min_d, "Orientation minimum delta cutoff");

	u_var_add_gui_header(&t, NULL, "Prediction");
	u_var_add_combo(&t, &t.pred_combo, "Prediction Type");
	u_var_add_i32(&t, &t.dbg_pred_every, "Debug prediction skips (try 30)");
	u_var_add_ro_ff_vec3_f32(&t, t.gyro_ff, "Gyroscope");
	u_var_add_ro_ff_vec3_f32(&t, t.accel_ff, "Accelerometer");
	u_var_add_f32(&t, &t.gravity_correction.z, "Gravity Correction");
	for (size_t i = 0; i < t.ui_sink.size(); i++) {
		char label[64] = {0};
		snprintf(label, sizeof(label), "Camera %zu", i);
		u_var_add_sink_debug(&t, &t.ui_sink[i], label);
	}

	u_var_add_gui_header(&t, NULL, "Stats");
	u_var_add_ro_raw_text(&t, "\nRecord to CSV files", "Record to CSV files");
	u_var_add_bool(&t, &t.slam_traj_writer->enabled, "Record tracked trajectory");
	u_var_add_bool(&t, &t.pred_traj_writer->enabled, "Record predicted trajectory");
	u_var_add_bool(&t, &t.filt_traj_writer->enabled, "Record filtered trajectory");
	u_var_add_bool(&t, &t.slam_times_writer->enabled, "Record tracker times");
	u_var_add_bool(&t, &t.slam_features_writer->enabled, "Record feature count");
	timing_ui_setup(t);
	features_ui_setup(t);
	// Later, gt_ui_setup will setup the tracking error UI if ground truth becomes available
}

static void
add_camera_calibration(const TrackerSlam &t, const t_slam_camera_calibration *calib, uint32_t cam_index)
{
	const t_camera_calibration &view = calib->base;

	vit_camera_calibration params = {};
	params.camera_index = cam_index;
	params.width = view.image_size_pixels.w;
	params.height = view.image_size_pixels.h;
	params.frequency = calib->frequency;

	params.fx = view.intrinsics[0][0];
	params.fy = view.intrinsics[1][1];
	params.cx = view.intrinsics[0][2];
	params.cy = view.intrinsics[1][2];

	switch (view.distortion_model) {
	case T_DISTORTION_OPENCV_RADTAN_8: {
		params.model = VIT_CAMERA_DISTORTION_RT8;
		const size_t size = sizeof(struct t_camera_calibration_rt8_params) + sizeof(double);
		params.distortion_count = size / sizeof(double);
		SLAM_ASSERT_(params.distortion_count == 9);

		memcpy(params.distortion, &view.rt8, size);

		// -1 metric radius tells Basalt to estimate the metric radius on its own.
		params.distortion[8] = -1.f;
		break;
	}
	case T_DISTORTION_WMR: {
		params.model = VIT_CAMERA_DISTORTION_RT8;
		const size_t size = sizeof(struct t_camera_calibration_rt8_params) + sizeof(double);
		params.distortion_count = size / sizeof(double);
		SLAM_ASSERT_(params.distortion_count == 9);

		memcpy(params.distortion, &view.wmr, size);

		params.distortion[8] = view.wmr.rpmax;

		break;
	}
	case T_DISTORTION_FISHEYE_KB4: {
		params.model = VIT_CAMERA_DISTORTION_KB4;
		const size_t size = sizeof(struct t_camera_calibration_kb4_params);
		params.distortion_count = size / sizeof(double);
		SLAM_ASSERT_(params.distortion_count == 4);

		memcpy(params.distortion, &view.kb4, size);
		break;
	}
	default:
		SLAM_ASSERT(false, "SLAM doesn't support distortion type %s",
		            t_stringify_camera_distortion_model(view.distortion_model));
		break;
	}

	xrt_matrix_4x4 T; // Row major T_imu_cam
	math_matrix_4x4_transpose(&calib->T_imu_cam, &T);

	// Converts the xrt_matrix_4x4 from float to double
	for (size_t i = 0; i < ARRAY_SIZE(params.transform); ++i)
		params.transform[i] = T.v[i];

	vit_result_t vres = t.vit.tracker_add_camera_calibration(t.tracker, &params);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to add camera calibration for camera %u", cam_index);
	}
}

static void
add_imu_calibration(const TrackerSlam &t, const t_slam_imu_calibration *imu_calib)
{
	vit_imu_calibration_t params = {};
	params.imu_index = 0;
	params.frequency = imu_calib->frequency;

	// TODO improve memcpy size calculation

	const t_inertial_calibration &accel = imu_calib->base.accel;
	memcpy(params.accel.transform, accel.transform, sizeof(double) * 9);
	memcpy(params.accel.offset, accel.offset, sizeof(double) * 3);
	memcpy(params.accel.bias_std, accel.bias_std, sizeof(double) * 3);
	memcpy(params.accel.noise_std, accel.noise_std, sizeof(double) * 3);

	const t_inertial_calibration &gyro = imu_calib->base.gyro;
	memcpy(params.gyro.transform, gyro.transform, sizeof(double) * 9);
	memcpy(params.gyro.offset, gyro.offset, sizeof(double) * 3);
	memcpy(params.gyro.bias_std, gyro.bias_std, sizeof(double) * 3);
	memcpy(params.gyro.noise_std, gyro.noise_std, sizeof(double) * 3);

	vit_result_t vres = t.vit.tracker_add_imu_calibration(t.tracker, &params);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to add imu calibration");
	}
}

static void
send_calibration(const TrackerSlam &t, const t_slam_calibration &c)
{
	// Try to send camera calibration data to the SLAM system
	if (t.exts.has_add_camera_calibration) {
		for (int i = 0; i < c.cam_count; i++) {
			SLAM_INFO("Sending Camera %d calibration from Monado", i);
			add_camera_calibration(t, &c.cams[i], i);
		}
	} else {
		SLAM_WARN("Tracker doesn't support camera calibration");
	}

	// Try to send IMU calibration data to the SLAM system
	if (t.exts.has_add_imu_calibration) {
		SLAM_INFO("Sending IMU calibration from Monado");
		add_imu_calibration(t, &c.imu);
	} else {
		SLAM_WARN("Tracker doesn't support IMU calibration");
	}
}

} // namespace xrt::auxiliary::tracking::slam

using namespace xrt::auxiliary::tracking::slam;

/*
 *
 * External functions
 *
 */

//! Get a filtered prediction from the SLAM tracked poses.
extern "C" void
t_slam_get_tracked_pose(struct xrt_tracked_slam *xts, timepoint_ns when_ns, struct xrt_space_relation *out_relation)
{
	XRT_TRACE_MARKER();

	auto &t = *container_of(xts, TrackerSlam, base);

	//! @todo This should not be cached, the same timestamp can be requested at a
	//! later time on the frame for a better prediction.
	if (when_ns == t.last_ts) {
		*out_relation = t.last_rel;
		return;
	}

	{
		std::shared_lock<std::shared_mutex> vit_guard(t.vit_lock); // 0104
		if (!t.stopped) {
			flush_poses(t);
		} else {
			t.dropped_after_stop++;
		}
	}

	// reverb-g2 0102: sample the anchor age this prediction is about to bridge. Under
	// correction.mutex: this function is called from the compositor AND, with
	// WMR_CONSTELLATION_CONTROLLERS=1, from the constellation tracker's own thread (see the
	// mutex's comment) -- the first version of this block assumed one thread and raced.
	if (t.age.every > 0) {
		int64_t rel_ts = 0;
		xrt_space_relation rel{};
		unique_lock age_lock(t.correction.mutex);
		if (t.slam_rels.get_latest(&rel_ts, &rel)) {
			t.age.ms[t.age.n % 1024] = (float)(when_ns - rel_ts) / 1e6f;
			t.age.n++;
			if (t.age.n % t.age.every == 0) {
				int cnt = t.age.n < 1024 ? t.age.n : 1024;
				std::vector<float> v(t.age.ms, t.age.ms + cnt);
				std::sort(v.begin(), v.end());
				SLAM_INFO("pose age ms: p50 %.1f p90 %.1f max %.1f over %d predictions", v[cnt / 2],
				          v[(cnt * 9) / 10], v[cnt - 1], cnt);
			}
		}
	}

	predict_pose(t, when_ns, out_relation);
	t.pred_traj_writer->push({when_ns, out_relation->pose});

	apply_correction_spread(t, out_relation);

	filter_pose(t, when_ns, out_relation);
	t.filt_traj_writer->push({when_ns, out_relation->pose});

	t.last_rel = *out_relation;
	t.last_ts = when_ns;

	if (t.gt.override_tracking) {
		out_relation->pose = gt2xr_pose(t.gt.origin, get_gt_pose_at(*t.gt.trajectory, when_ns));
	}
}

//! Receive and register ground truth to use for trajectory error metrics.
extern "C" void
t_slam_gt_sink_push(struct xrt_pose_sink *sink, xrt_pose_sample *sample)
{
	XRT_TRACE_MARKER();

	auto &t = *container_of(sink, TrackerSlam, gt_sink);

	if (t.gt.trajectory->empty()) {
		t.gt.origin = sample->pose;
		gt_ui_setup(t);
	}

	t.gt.trajectory->insert_or_assign(sample->timestamp_ns, sample->pose);
	xrt_sink_push_pose(t.euroc_recorder->gt, sample);
}

//! Receive and register masks to use in the next image
extern "C" void
t_slam_hand_mask_sink_push(struct xrt_hand_masks_sink *sink, struct xrt_hand_masks_sample *hand_masks)
{
	XRT_TRACE_MARKER();

	auto &t = *container_of(sink, TrackerSlam, hand_masks_sink);
	unique_lock lock(t.last_hand_masks_mutex);
	t.last_hand_masks = *hand_masks;
}

//! Receive and send IMU samples to the external SLAM system
extern "C" void
t_slam_receive_imu(struct xrt_imu_sink *sink, struct xrt_imu_sample *s)
{
	XRT_TRACE_MARKER();

	auto &t = *container_of(sink, TrackerSlam, imu_sink);

	timepoint_ns ts = s->timestamp_ns;
	xrt_vec3_f64 a = s->accel_m_s2;
	xrt_vec3_f64 w = s->gyro_rad_secs;

	timepoint_ns now = (timepoint_ns)os_monotonic_get_ns();
	SLAM_TRACE("[%ld] imu t=%ld  a=[%f,%f,%f] w=[%f,%f,%f]", now, ts, a.x, a.y, a.z, w.x, w.y, w.z);
	// Check monotonically increasing timestamps
	if (ts <= t.last_imu_ts) {
		SLAM_WARN("Sample (%" PRId64 ") is older than last (%" PRId64 ") by %" PRId64 " ns", ts, t.last_imu_ts,
		          t.last_imu_ts - ts);
		return;
	}
	t.last_imu_ts = ts;

	//! @todo There are many conversions like these between xrt and
	//! slam_tracker.hpp types. Implement a casting mechanism to avoid copies.
	vit_imu_sample_t sample = {};
	sample.timestamp = ts;
	sample.ax = a.x;
	sample.ay = a.y;
	sample.az = a.z;
	sample.wx = w.x;
	sample.wy = w.y;
	sample.wz = w.z;

	if (t.submit) {
		std::shared_lock<std::shared_mutex> vit_guard(t.vit_lock); // 0104
		if (!t.stopped) {
			t.vit.tracker_push_imu_sample(t.tracker, &sample);
		} else {
			t.dropped_after_stop++;
		}
	}

	xrt_sink_push_imu(t.euroc_recorder->imu, s);

	struct xrt_vec3 gyro = {(float)w.x, (float)w.y, (float)w.z};
	struct xrt_vec3 accel = {(float)a.x, (float)a.y, (float)a.z};
	os_mutex_lock(&t.lock_ff);
	m_ff_vec3_f32_push(t.gyro_ff, &gyro, ts);
	m_ff_vec3_f32_push(t.accel_ff, &accel, ts);
	os_mutex_unlock(&t.lock_ff);
}

//! Push the frame to the external SLAM system
static void
receive_frame(TrackerSlam &t, struct xrt_frame *frame, uint32_t cam_index)
{
	XRT_TRACE_MARKER();

	SLAM_DASSERT_(frame->timestamp < INT64_MAX);

	// Return early if we don't submit
	if (!t.submit) {
		return;
	}

	// 0104: held for the rest of this function (flush_poses + the image push below).
	std::shared_lock<std::shared_mutex> vit_guard(t.vit_lock);
	if (t.stopped) {
		t.dropped_after_stop++;
		return;
	}

	if (cam_index == t.cam_count - 1) {
		flush_poses(t); // Useful to flush SLAM poses when no openxr app is open
	}

	SLAM_DASSERT(t.last_cam_ts[0] != INT64_MIN || cam_index == 0, "First frame was not a cam0 frame");

	// Check monotonically increasing timestamps
	timepoint_ns &last_ts = t.last_cam_ts[cam_index];
	timepoint_ns ts = (int64_t)frame->timestamp;
	SLAM_TRACE("[%" PRId64 "] cam%d frame t=%" PRId64, os_monotonic_get_ns(), cam_index, ts);
	// A non-monotonic timestamp used to be warned about and then pushed anyway. That is not
	// survivable downstream: Basalt asserts on it and takes the whole process down
	// (sqrt_keypoint_vio.cpp:311 "frame timestamps not monotonically increasing?!" -> SIGABRT,
	// killing monado-service mid-session). Seen three times on 2026-08-12 with three different
	// triggers -- disk I/O from the EuRoC recorder, CPU load from denser feature detection, and
	// a `CamerasDmaReset` burst from the headset itself -- so it is not an exotic condition, it
	// is whatever makes the camera clock hiccup.
	//
	// Drop the frame instead. It has to be dropped as a whole BUNDLE: the tracker requires every
	// camera in one bundle to carry the same timestamp, so pushing cam1 after having dropped
	// cam0 trades one assert for another. cam0 decides, the rest follow.
	//
	// Note this stalls tracking until the camera clock passes the last accepted timestamp
	// again, which for a large backwards jump can be seconds. That is deliberate: the
	// alternative is feeding the tracker a frame older than the one it already integrated,
	// which is the crash. The stall is logged so it can never be silent.
	if (cam_index == 0) {
		bool drop = last_ts >= ts;
		// The backwards check alone is poisonable: one insane FORWARD timestamp gets
		// accepted (it is "newer"), becomes the high-water mark, and then every sane
		// frame is dropped as old -- forever, with tracking silently blind. Reject
		// forward jumps beyond any plausible frame gap without updating the mark; a
		// real clock re-baseline (e.g. resume after a long stall) is recognized by the
		// next frame being consistent with the jumped clock, at the cost of one frame.
		constexpr timepoint_ns FWD_JUMP_LIMIT_NS = 10LL * U_TIME_1S_IN_NS;
		constexpr timepoint_ns FWD_CONFIRM_WINDOW_NS = 1LL * U_TIME_1S_IN_NS;
		if (!drop && last_ts != INT64_MIN && ts - last_ts > FWD_JUMP_LIMIT_NS) {
			timepoint_ns since_candidate = ts - t.pending_fwd_jump_ts;
			if (t.pending_fwd_jump_ts != INT64_MIN && since_candidate >= 0 &&
			    since_candidate < FWD_CONFIRM_WINDOW_NS) {
				SLAM_WARN("Camera clock re-baselined forward by %" PRId64
				          " ns (confirmed by consecutive frames), accepting",
				          ts - last_ts);
				t.pending_fwd_jump_ts = INT64_MIN;
			} else {
				SLAM_WARN("Rejecting insane forward jump: cam0 frame (%" PRId64
				          ") is %" PRId64 " ns ahead of last accepted (%" PRId64
				          ") -- dropping bundle, keeping baseline",
				          ts, ts - last_ts, last_ts);
				t.pending_fwd_jump_ts = ts;
				drop = true;
			}
		} else if (!drop) {
			t.pending_fwd_jump_ts = INT64_MIN;
		}
		if (drop && !t.dropping_bundle) {
			t.dropped_bundles = 0;
		}
		t.dropping_bundle = drop;
	}
	if (t.dropping_bundle) {
		if (cam_index == 0) {
			if (t.dropped_bundles % 30 == 0) { // ~1 s of camera frames between messages
				SLAM_WARN("Dropping frame bundles: cam0 frame (%" PRId64
				          ") is older than the last accepted (%" PRId64 ") by %" PRId64
				          " ns (%d dropped so far)",
				          ts, last_ts, last_ts - ts, t.dropped_bundles + 1);
			}
			t.dropped_bundles++;
		}
		return;
	}
	if (t.dropped_bundles > 0 && cam_index == 0) {
		SLAM_INFO("Camera clock recovered after dropping %d frame bundles", t.dropped_bundles);
		t.dropped_bundles = 0;
	}
	if (last_ts >= ts) {
		// Not cam0, so the bundle was already accepted: warn but keep the timestamp coherent
		// with cam0's rather than dropping half a bundle.
		SLAM_WARN("Frame (%" PRId64 ") is older than last (%" PRId64 ") by %" PRId64 " ns", ts, last_ts,
		          last_ts - ts);
	}
	last_ts = ts;

	// Construct and send the image sample
	vit_img_sample sample = {};
	sample.cam_index = cam_index;
	sample.timestamp = ts;

	sample.data = frame->data;
	sample.width = frame->width;
	sample.height = frame->height;
	sample.stride = frame->stride;
	sample.size = frame->size;

	// TODO check format before
	switch (frame->format) {
	case XRT_FORMAT_L8: sample.format = VIT_IMAGE_FORMAT_L8; break;
	case XRT_FORMAT_R8G8B8: sample.format = VIT_IMAGE_FORMAT_R8G8B8; break;
	default: SLAM_ERROR("Unknown image format"); return;
	}

	xrt_hand_masks_sample hand_masks{};
	{
		unique_lock lock(t.last_hand_masks_mutex);
		hand_masks = t.last_hand_masks;
	}

	auto &view = hand_masks.views[cam_index];
	std::vector<vit_mask_t> masks;
	if (view.enabled) {
		for (auto &hand : view.hands) {
			if (!hand.enabled) {
				continue;
			}
			vit_mask_t mask{};
			mask.x = hand.rect.x;
			mask.y = hand.rect.y;
			mask.w = hand.rect.w;
			mask.h = hand.rect.h;
			masks.push_back(mask);
		}

		sample.mask_count = masks.size();
		sample.masks = masks.empty() ? nullptr : masks.data();
	}

	{
		XRT_TRACE_IDENT(slam_push);
		t.vit.tracker_push_img_sample(t.tracker, &sample);
	}
}

#define DEFINE_RECEIVE_CAM(cam_id)                                                                                     \
	extern "C" void t_slam_receive_cam##cam_id(struct xrt_frame_sink *sink, struct xrt_frame *frame)               \
	{                                                                                                              \
		auto &t = *container_of(sink, TrackerSlam, cam_sinks[cam_id]);                                         \
		receive_frame(t, frame, cam_id);                                                                       \
		u_sink_debug_push_frame(&t.ui_sink[cam_id], frame);                                                    \
		xrt_sink_push_frame(t.euroc_recorder->cams[cam_id], frame);                                            \
	}

DEFINE_RECEIVE_CAM(0)
DEFINE_RECEIVE_CAM(1)
DEFINE_RECEIVE_CAM(2)
DEFINE_RECEIVE_CAM(3)
DEFINE_RECEIVE_CAM(4)

//! Define a function for each XRT_TRACKING_MAX_CAMS and reference it in this array
void (*t_slam_receive_cam[XRT_TRACKING_MAX_CAMS])(xrt_frame_sink *, xrt_frame *) = {
    t_slam_receive_cam0, //
    t_slam_receive_cam1, //
    t_slam_receive_cam2, //
    t_slam_receive_cam3, //
    t_slam_receive_cam4, //
};


extern "C" void
t_slam_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerSlam, node);
	if (t.ovr_tracker != NULL) {
		t_openvr_tracker_stop(t.ovr_tracker);
	}

	vit_result_t vres;
	{
		// 0104: no push or pose query may be inside the tracker while it stops, and none may
		// enter afterwards (the frame sources are broken apart AFTER this node -- see vit_lock).
		std::unique_lock<std::shared_mutex> vit_guard(t.vit_lock);
		t.stopped = true;
		vres = t.vit.tracker_stop(t.tracker);
	}
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to stop VIT tracker");
		return;
	}

	SLAM_DEBUG("SLAM tracker dismantled");
}

extern "C" void
t_slam_node_destroy(struct xrt_frame_node *node)
{
	auto t_ptr = container_of(node, TrackerSlam, node);
	auto &t = *t_ptr; // Needed by SLAM_DEBUG
	SLAM_DEBUG("Destroying SLAM tracker");
	if (t.dropped_after_stop > 0) {
		SLAM_WARN("0104: %u sample pushes / pose queries arrived after tracker_stop and were dropped "
		          "(each one was the pre-0104 pop_pose SIGSEGV)",
		          t.dropped_after_stop.load());
	}
	if (t.ovr_tracker != NULL) {
		t_openvr_tracker_destroy(t.ovr_tracker);
	}
	delete t.gt.trajectory;
	delete t.slam_times_writer;
	delete t.slam_features_writer;
	delete t.slam_traj_writer;
	delete t.pred_traj_writer;
	delete t.filt_traj_writer;
	u_var_remove_root(t_ptr);
	for (size_t i = 0; i < t.ui_sink.size(); i++) {
		u_sink_debug_destroy(&t.ui_sink[i]);
	}
	m_ff_vec3_f32_free(&t.gyro_ff);
	m_ff_vec3_f32_free(&t.accel_ff);
	os_mutex_destroy(&t.lock_ff);
	m_ff_vec3_f32_free(&t.filter.pos_ff);
	m_ff_vec3_f32_free(&t.filter.rot_ff);

	t_ptr->vit.tracker_destroy(t_ptr->tracker);
	t_vit_bundle_unload(&t_ptr->vit);

	delete t_ptr;
}

extern "C" int
t_slam_start(struct xrt_tracked_slam *xts)
{
	auto &t = *container_of(xts, TrackerSlam, base);
	vit_result_t vres = t.vit.tracker_start(t.tracker);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to start VIT tracker");
		return -1;
	}

	SLAM_DEBUG("SLAM tracker started");
	return 0;
}

extern "C" void
t_slam_fill_default_config(struct t_slam_tracker_config *config)
{
	config->log_level = debug_get_log_option_slam_log();
	config->vit_system_library_path = debug_get_option_vit_system_library_path();
	config->slam_config = debug_get_option_slam_config();
	config->slam_ui = debug_get_bool_option_slam_ui();
	config->submit_from_start = debug_get_bool_option_slam_submit_from_start();
	config->openvr_groundtruth_device = int(debug_get_num_option_slam_openvr_groundtruth_device());
	config->prediction = t_slam_prediction_type(debug_get_num_option_slam_prediction_type());
	config->write_csvs = debug_get_bool_option_slam_write_csvs();
	config->csv_path = debug_get_option_slam_csv_path();
	config->timing_stat = debug_get_bool_option_slam_timing_stat();
	config->features_stat = debug_get_bool_option_slam_features_stat();
	config->cam_count = int(debug_get_num_option_slam_cam_count());
	config->slam_calib = NULL;
}

extern "C" int
t_slam_create(struct xrt_frame_context *xfctx,
              struct t_slam_tracker_config *config,
              struct xrt_tracked_slam **out_xts,
              struct xrt_slam_sinks **out_sink)
{
	struct t_slam_tracker_config default_config = {};
	if (config == nullptr) {
		t_slam_fill_default_config(&default_config);
		config = &default_config;
	}

	enum u_logging_level log_level = config->log_level;

	std::unique_ptr<TrackerSlam> t_ptr = std::make_unique<TrackerSlam>();
	TrackerSlam &t = *t_ptr;

	t.log_level = log_level;

	SLAM_INFO("Loading VIT system library from VIT_SYSTEM_LIBRARY_PATH='%s'", config->vit_system_library_path);

	if (!t_vit_bundle_load(&t.vit, config->vit_system_library_path)) {
		SLAM_ERROR("Failed to load VIT system library from '%s'", config->vit_system_library_path);
		return -1;
	}

	// Check the user has provided a SLAM_CONFIG file
	const char *config_file = config->slam_config;
	bool some_calib = config->slam_calib != nullptr;
	if (!config_file && !some_calib) {
		SLAM_WARN("Unable to determine sensor calibration, did you forget to set SLAM_CONFIG?");
		return -1;
	}

	struct vit_config system_config = {};
	system_config.file = config_file;
	system_config.cam_count = config->cam_count;
	system_config.show_ui = config->slam_ui;

	vit_result_t vres = t.vit.tracker_create(&system_config, &t.tracker);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to create VIT tracker (%d)", vres);
		return -1;
	}

	vres = t.vit.tracker_get_supported_extensions(t.tracker, &t.exts);
	if (vres != VIT_SUCCESS) {
		SLAM_ERROR("Failed to get VIT tracker supported extensions (%d)", vres);
		return -1;
	}

	t.base.get_tracked_pose = t_slam_get_tracked_pose;

	// SLAM_CONFIG is normally all-or-nothing: pass a file and the driver's calibration is no
	// longer sent, so tuning one pipeline parameter means hand-writing the whole device
	// calibration in the tracker's own format first. SLAM_CONFIG_PIPELINE_ONLY=1 says the file
	// carries pipeline settings only, and keeps the driver calibration -- which is what makes a
	// config sweep against real hardware a one-line change per run instead of a project.
	bool pipeline_only = debug_get_bool_option_slam_config_pipeline_only();
	if (!config_file) {
		SLAM_INFO("Using calibration from driver and default pipeline settings");
		send_calibration(t, *config->slam_calib); // Not null because of `some_calib`
	} else if (pipeline_only && some_calib) {
		SLAM_INFO("Using calibration from driver and pipeline settings from the SLAM_CONFIG file");
		send_calibration(t, *config->slam_calib);
	} else {
		SLAM_INFO("Using sensor calibration provided by the SLAM_CONFIG file");
	}

	SLAM_ASSERT(t_slam_receive_cam[ARRAY_SIZE(t_slam_receive_cam) - 1] != nullptr, "See `cam_sink_push` docs");
	t.sinks.cam_count = config->cam_count;
	for (int i = 0; i < XRT_TRACKING_MAX_CAMS; i++) {
		t.cam_sinks[i].push_frame = t_slam_receive_cam[i];
		t.sinks.cams[i] = &t.cam_sinks[i];
	}

	t.imu_sink.push_imu = t_slam_receive_imu;
	t.sinks.imu = &t.imu_sink;

	t.gt_sink.push_pose = t_slam_gt_sink_push;
	t.sinks.gt = &t.gt_sink;

	t.hand_masks_sink.push_hand_masks = t_slam_hand_mask_sink_push;
	t.sinks.hand_masks = &t.hand_masks_sink;

	t.submit = config->submit_from_start;
	t.cam_count = config->cam_count;

	t.node.break_apart = t_slam_node_break_apart;
	t.node.destroy = t_slam_node_destroy;

	xrt_frame_context_add(xfctx, &t.node);

	// EUROC_RECORD=1 starts the EuRoC dataset recorder without needing the debug GUI, so a
	// dataset can be captured headlessly and replayed later through the euroc driver.
	t.euroc_recorder = euroc_recorder_create(xfctx, debug_get_option_euroc_record_path(), t.cam_count,
	                                         debug_get_bool_option_euroc_record());

	t.last_imu_ts = INT64_MIN;
	t.last_cam_ts = vector<timepoint_ns>(t.cam_count, INT64_MIN);
	t.last_hand_masks = xrt_hand_masks_sample{};

	t.pred_type = config->prediction;
	t.pred_freeze_position = debug_get_bool_option_slam_pred_freeze_position();
	t.pred_neck_arm_mm = debug_get_num_option_slam_pred_neck_arm_mm();
	t.pred_position_horizon_s = (double)debug_get_num_option_slam_pred_position_horizon_ms() / 1000.0;
	t.pred_position_max_speed_m_s = (double)debug_get_num_option_slam_pred_position_max_speed_cm_s() / 100.0;
	if (t.pred_freeze_position && t.pred_position_horizon_s > 0.0) {
		SLAM_INFO("Bounded position horizon on: extrapolating SLAM linear velocity for up to %.0f ms "
		          "(SLAM_PRED_POSITION_HORIZON_MS), clamped to %.2f m/s (SLAM_PRED_POSITION_MAX_SPEED_CM_S, "
		          "0 = no clamp). NOT YET HARDWARE-VALIDATED.",
		          t.pred_position_horizon_s * 1000.0, t.pred_position_max_speed_m_s);
	}

	// The three output filters below are off by default upstream and only reachable from the
	// debug GUI, so the pose handed to the application is raw SLAM output. Measured on a
	// Reverb G2 on 2026-08-12, in-game: consecutive poses differ by 0.89 mm at the median but
	// 152 mm at p99 and up to 582 mm -- occasional large jumps rather than smooth noise, which
	// is exactly what the wearer reports as jitter. SLAM_FILTER makes them selectable without
	// the GUI so the effect can be measured (raw trajectory lands in tracking.csv, the filtered
	// one the application actually receives lands in filtering.csv).
	const char *filter_name = debug_get_option_slam_filter();
	if (filter_name != NULL) {
		if (strcmp(filter_name, "one_euro") == 0) {
			t.filter.use_one_euro_filter = true;
		} else if (strcmp(filter_name, "moving_average") == 0) {
			t.filter.use_moving_average_filter = true;
		} else if (strcmp(filter_name, "exponential") == 0) {
			t.filter.use_exponential_smoothing_filter = true;
		} else if (strcmp(filter_name, "none") != 0) {
			SLAM_WARN("Unknown SLAM_FILTER '%s' (one_euro|moving_average|exponential|none)",
			          filter_name);
		}
		SLAM_INFO("Output filter: %s", filter_name);
	}

	t.auto_reset.enabled = debug_get_bool_option_slam_auto_reset();
	t.auto_reset.max_speed = (double)debug_get_num_option_slam_auto_reset_max_speed();
	t.auto_reset.quiet_until = 0;
	t.auto_reset.count = 0;
	if (t.auto_reset.enabled) {
		SLAM_INFO("Divergence auto-reset on, threshold %.0f m/s (integer env var; SLAM_AUTO_RESET=0 disables)",
		          t.auto_reset.max_speed);
	}

	t.reset_offset.enabled = debug_get_bool_option_slam_reset_offset_carry();
	if (t.reset_offset.enabled) {
		SLAM_INFO("Divergence-reset frame continuity on: an auto-reset above will carry its offset "
		          "into the output pose instead of teleporting (SLAM_RESET_OFFSET_CARRY=0 disables)");
	}

	long correction_spread_ms = debug_get_num_option_slam_correction_spread_ms();
	t.correction.enabled = correction_spread_ms > 0;
	t.correction.spread_s = (double)correction_spread_ms / 1000.0;
	if (t.correction.enabled) {
		SLAM_INFO("Correction spreading on: anchor-arrival jumps folded into a %ld ms decaying "
		          "offset on the delivered pose instead of landing in one frame "
		          "(SLAM_CORRECTION_SPREAD_MS=0 disables)",
		          correction_spread_ms);
	}
	long correction_avg_n = debug_get_num_option_slam_correction_avg_n();
	t.correction.avg_n = (int)(correction_avg_n < 1 ? 1 : (correction_avg_n > 8 ? 8 : correction_avg_n));
	if (t.correction.enabled && t.correction.avg_n > 1) {
		SLAM_INFO("Correction input averaged over the last %d anchors (SLAM_CORRECTION_AVG_N)", t.correction.avg_n);
	}
	t.age.every = (int)debug_get_num_option_slam_pose_age_log();
	if (t.age.every > 0) {
		SLAM_INFO("Pose age log on: anchor age p50/p90/max every %d predictions (SLAM_POSE_AGE_LOG)", t.age.every);
	}

	long session_anchor_radius_cm = debug_get_num_option_slam_session_anchor_radius_cm();
	t.session_anchor.enabled = session_anchor_radius_cm > 0;
	t.session_anchor.radius_m = (double)session_anchor_radius_cm / 100.0;
	if (t.session_anchor.enabled && !t.reset_offset.enabled) {
		// This guard's anchor is only meaningful in a continuous output frame, which only
		// reset_offset guarantees (SLAM_RESET_OFFSET_CARRY=0) -- see the guard's own
		// caveat comment in flush_poses. Rather than silently no-op, say so.
		SLAM_WARN("SLAM_SESSION_ANCHOR_RADIUS_CM is set but SLAM_RESET_OFFSET_CARRY=0 -- the "
		          "session-anchor guard needs reset_offset's frame continuity to mean anything "
		          "and will NOT run until SLAM_RESET_OFFSET_CARRY is enabled (the default).");
		t.session_anchor.enabled = false;
	}
	if (t.session_anchor.enabled) {
		SLAM_INFO("Session-anchor divergence guard on: resetting if the output position ever "
		          "drifts more than %.2f m from the session's first accepted pose "
		          "(SLAM_SESSION_ANCHOR_RADIUS_CM=0 disables). NOT YET HARDWARE-VALIDATED -- "
		          "pending a wearer A/B, see TrackerSlam::session_anchor.",
		          t.session_anchor.radius_m);
	}

	t.quat_norm_check.enabled = debug_get_num_option_slam_quat_norm_check() != 0;
	if (t.quat_norm_check.enabled) {
		SLAM_INFO("Orientation quaternion norm sanity guard on: resetting if a pose's "
		          "orientation quaternion norm falls outside [0.5, 1.5] "
		          "(SLAM_QUAT_NORM_CHECK=0 disables). NOT YET HARDWARE-VALIDATED -- pending "
		          "a wearer A/B, see TrackerSlam::quat_norm_check.");
	}

	t.filter.one_euro_before_predict = debug_get_bool_option_slam_filter_before_predict();
	SLAM_INFO("One euro filter runs %s prediction (SLAM_FILTER_BEFORE_PREDICT=%d)",
	          t.filter.one_euro_before_predict ? "BEFORE" : "after", t.filter.one_euro_before_predict);

	t.filter.pos_min_cutoff = debug_get_float_option_slam_filter_pos_min_cutoff();
	t.filter.rot_min_cutoff = debug_get_float_option_slam_filter_rot_min_cutoff();
	t.filter.min_dcutoff = debug_get_float_option_slam_filter_min_dcutoff();
	t.filter.beta = debug_get_float_option_slam_filter_beta();
	t.filter.pos_deadzone_m = debug_get_float_option_slam_pos_deadzone();
	if (t.filter.pos_deadzone_m > 0.0f) {
		SLAM_INFO("Position deadband ON: %.1f mm (SLAM_POS_DEADZONE_M) -- measurements taken "
		          "through this are NOT raw",
		          (double)t.filter.pos_deadzone_m * 1000.0);
	}
	SLAM_INFO("One euro: pos cutoff %.2f Hz, rot cutoff %.2f Hz, beta %.2f, dcutoff %.2f Hz",
	          (double)t.filter.pos_min_cutoff, (double)t.filter.rot_min_cutoff, (double)t.filter.beta,
	          (double)t.filter.min_dcutoff);

	m_filter_euro_vec3_init(&t.filter.pos_oe, t.filter.pos_min_cutoff, t.filter.min_dcutoff, t.filter.beta);
	m_filter_euro_quat_init(&t.filter.rot_oe, t.filter.rot_min_cutoff, t.filter.min_dcutoff, t.filter.beta);

	t.gt.trajectory = new Trajectory{};

	// timing.csv's columns MUST be populated before TimingWriter is constructed just below:
	// CSVWriter's constructor copies the vector it's handed BY VALUE into its own
	// column_names member, so whatever t.timing.columns held at that instant is what the
	// CSV header uses forever after. setup_ui() -> timing_ui_setup() (further down) is the
	// only thing that used to populate t.timing.columns, and it runs AFTER this point --
	// meaning TimingWriter's own column_names was permanently empty (bare "#" header, first
	// data row glued onto it) unless the debug GUI's timing button had already been clicked,
	// for the whole history of this feature. Worse, SLAM_TIMING_STAT (config->timing_stat)
	// was read into config since day one and never consumed anywhere -- a dangling knob that
	// looked like it should have been fixing exactly this (T202/docs follow-up 2026-08-17).
	// Populate the columns now, and negotiate the extension headlessly when requested, so a
	// service with no debug GUI attached still gets Basalt's full per-stage breakdown.
	// timing_ui_setup() still owns the GUI toggle and is free to flip t.timing.enabled later;
	// it recomputes the same columns (idempotent) so the two paths can't disagree.
	timing_columns_setup(t);
	if (t.exts.has_pose_timing && config->timing_stat) {
		vit_result_t tres = t.vit.tracker_enable_extension(t.tracker, VIT_TRACKER_EXTENSION_POSE_TIMING, true);
		if (tres != VIT_SUCCESS) {
			SLAM_ERROR("Failed to enable the tracker pose-timing extension (%d)", tres);
		} else {
			t.timing.enabled = true;
			SLAM_INFO("Pose timing enabled headlessly via SLAM_TIMING_STAT");
		}
	}

	// Setup CSV files
	bool write_csvs = config->write_csvs;
	string dir = config->csv_path;
	t.slam_times_writer = new TimingWriter(dir, "timing.csv", write_csvs, t.timing.columns);
	t.slam_features_writer = new FeaturesWriter(dir, "features.csv", write_csvs, t.cam_count);
	t.slam_traj_writer = new TrajectoryWriter(dir, "tracking.csv", write_csvs);
	t.pred_traj_writer = new TrajectoryWriter(dir, "prediction.csv", write_csvs);
	t.filt_traj_writer = new TrajectoryWriter(dir, "filtering.csv", write_csvs);

	setup_ui(t);

	// Setup OpenVR groundtruth tracker
	if (config->openvr_groundtruth_device > 0) {
		enum openvr_device dev_class = openvr_device(config->openvr_groundtruth_device);
		const double freq = 1000.0f;
		t.ovr_tracker = t_openvr_tracker_create(freq, &dev_class, &t.sinks.gt, 1);
		if (t.ovr_tracker != NULL) {
			t_openvr_tracker_start(t.ovr_tracker);
		}
	}

	// Get ownership
	TrackerSlam *tracker = t_ptr.release();

	*out_xts = &tracker->base;
	*out_sink = &tracker->sinks;

	SLAM_DEBUG("SLAM tracker created");
	return 0;
}
