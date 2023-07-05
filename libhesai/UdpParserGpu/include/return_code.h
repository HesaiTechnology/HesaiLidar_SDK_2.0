/*
[Hesai Technology Co., Ltd.] ("COMPANY") CONFIDENTIAL
Copyright (C) 2021 Hesai Technology Co., Ltd. All Rights Reserved.

NOTICE: All information contained herein is, and remains the property of
COMPANY. The intellectual and technical concepts contained herein are
proprietary to COMPANY and may be covered by Chinese and/or Foreign Patents,
patents in process, and are protected by trade secret and applicable
copyright laws. Dissemination of this information or reproduction of this
material is strictly forbidden unless prior written permission is obtained
from COMPANY. Access to the source code contained herein is hereby forbidden
to anyone except current COMPANY employees, managers or other third parties
who have executed Confidentiality and Non-disclosure agreements explicitly
covering such access.

The copyright notice above does not evidence any actual or intended
publication or disclosure of this source code, which includes information
that is confidential and/or proprietary, and is a trade secret, of COMPANY.

ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC
DISPLAY OF OR THROUGH USE OF THIS SOURCE CODE WITHOUT THE EXPRESS WRITTEN
CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
LAWS AND INTERNATIONAL TREATIES. THE RECEIPT OR POSSESSION OF THIS SOURCE
CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO
REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR
SELL ANYTHING THAT IT MAY DESCRIBE, IN WHOLE OR IN PART.
*/

// DO NOT change this file !!!!!
#ifndef __RETURE_CODE__
#define __RETURE_CODE__

#include <vector>
#include <cinttypes>
#include <memory>
namespace hesai
{
namespace lidar
{
enum class ReturnCode : int {
  Default = 0,
  CorrectionsUnloaded = 1,
  ConfigJsonUnloaded = 2,
  LiDAR2SpeedometerCalibrationUnloaded = 3,
  LiDAR2GroundCalibrationUnloaded = 4,
  CudaMemcpyHostToDeviceError = 5,
  CudaMemcpyDeviceToHostError = 6,
  LiDARPacketInvalid = 7,
  LiDARPacketExpired = 8,
  LiDARPacketNewFrame = 9,
  LiDARPacketPreFrame = 10,
  LiDARPacketPostFrame = 11,
  PerceptionHistoryReset = 12,
  WaitForNextFrame = 13,
  LiDARRawFrameNotReady = 14,
  FrameScansTooFew = 15,
  CudaXYZComputingError = 16,
  CudaMotionCompensationError = 17,
  GravityDirectionHistoryEmpty = 18,
  GravityDirectionHistoryExpired = 19,
  GravityExpired = 20,
  RPYRateHistoryEmpty = 21,
  VelocityHistoryEmpty = 22,
  RPYRateHistoryExpired = 23,
  VelocityHistoryExpired = 24,
  SpeedometerExpired = 25,
  RPYRateStuck = 26,
  VelocityStuck = 27,
  SpeedometerStuck = 28,
  CudaPointsClusteringError = 29,
  CudaProposeObstacleCuboidsError = 30,
  CudaComputeCuboidTimestampError = 31,
  DetectionInferenceError = 32,
  SegmentationInferenceError = 33,
  Total
};
}
}
#endif  // !__AT128Perception_RETURE_CODE__
