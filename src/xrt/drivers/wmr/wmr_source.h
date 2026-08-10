// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface for WMR data sources
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup drv_wmr
 */

#pragma once

#include "wmr_config.h"
#include "xrt/xrt_frameserver.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"

/*!
 * WMR video/IMU data sources
 *
 * @addtogroup drv_wmr
 * @{
 */


#ifdef __cplusplus
extern "C" {
#endif

struct t_blob_sink;

/*!
 * Create and return the data source as a @ref xrt_fs ready for data streaming.
 *
 * @param ctrl_cam_constellation_sinks Optional, one @ref t_blob_sink per tracking camera (same
 *                                     indexing as @p cfg.tcams), or NULL. When non-NULL, each
 *                                     camera's controller-tracking blob detections are forwarded
 *                                     there in addition to the "Controller Blob Cam N" debug
 *                                     panel. Entries may individually be NULL.
 */
struct xrt_fs *
wmr_source_create(struct xrt_frame_context *xfctx,
                  struct xrt_prober_device *dev_holo,
                  struct wmr_hmd_config cfg,
                  struct t_blob_sink **ctrl_cam_constellation_sinks);

//! @todo IMU data should be generated from within the data source, but right
//! now we need this function because it is being generated from wmr_hmd
//! @todo Should this method receive raw or calibrated samples? Currently
//! receiving raw because Basalt can calibrate them, but other systems can't.
void
wmr_source_push_imu_packet(struct xrt_fs *xfs, timepoint_ns t, struct xrt_vec3 accel, struct xrt_vec3 gyro);

/*!
 * @}
 */

#ifdef __cplusplus
}
#endif
