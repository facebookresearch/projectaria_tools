/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <data_provider/json_io/FrontendTypesJson.h>
#include <data_provider/json_io/JsonHelpers.h>

namespace projectaria::tools::json {

using json = nlohmann::json;
using namespace projectaria::tools::data_provider;

namespace {
// Parsing for Camera online calibration
json projectionModelParametersToJson(const ProjectionModelParameters<float>& params) {
  return json{
      {"type", params.type},
      {"intrinsics", params.intrinsics},
      {"readoutTimeSec", params.readoutTimeSec},
      {"maxRadiusSquared", params.maxRadiusSquared},
      {"imageSize", {{"width", params.imageSize.width}, {"height", params.imageSize.height}}}};
}

ProjectionModelParameters<float> projectionModelParametersFromJson(const json& jsonObj) {
  ProjectionModelParameters<float> params;
  params.type = jsonObj.at("type").get<std::string>();
  params.intrinsics = jsonObj.at("intrinsics").get<std::vector<float>>();
  params.readoutTimeSec = jsonObj.at("readoutTimeSec").get<float>();
  params.maxRadiusSquared = jsonObj.at("maxRadiusSquared").get<float>();
  params.imageSize.width = jsonObj.at("imageSize").at("width").get<int16_t>();
  params.imageSize.height = jsonObj.at("imageSize").at("height").get<int16_t>();
  return params;
}

// Parsing for IMU online calibration
json imuMeasurementModelParametersToJson(const ImuMeasurementModelParameters<float>& params) {
  return json{
      {"gyroScaleVec", eigenVectorToJson(params.gyroScaleVec)},
      {"accelScaleVec", eigenVectorToJson(params.accelScaleVec)},
      {"accelBiasMSec2", eigenVectorToJson(params.accelBiasMSec2)},
      {"gyroBiasRadSec", eigenVectorToJson(params.gyroBiasRadSec)},
      {"accelNonorth", eigenMatrixToJson(params.accelNonorth)},
      {"gyroNonorth", eigenMatrixToJson(params.gyroNonorth)},
      {"gyroGSensitivityRadSecPerMSec2", eigenMatrixToJson(params.gyroGSensitivityRadSecPerMSec2)},
      {"dtReferenceAccelSec", params.dtReferenceAccelSec},
      {"dtReferenceGyroSec", params.dtReferenceGyroSec},
      {"T_Imu_BodyImu", se3ToJson(params.T_Imu_BodyImu)},
      {"nominalSamplingPeriodSec", params.nominalSamplingPeriodSec},
      {"accelSaturationThresholdMSec2", params.accelSaturationThresholdMSec2},
      {"gyroSaturationThresholdRadSec", params.gyroSaturationThresholdRadSec}};
}
ImuMeasurementModelParameters<float> imuMeasurementModelParametersFromJson(const json& jsonObj) {
  ImuMeasurementModelParameters<float> params;
  params.gyroScaleVec = eigenVectorFromJson<float, 3>(jsonObj.at("gyroScaleVec"));
  params.accelScaleVec = eigenVectorFromJson<float, 3>(jsonObj.at("accelScaleVec"));
  params.accelBiasMSec2 = eigenVectorFromJson<float, 3>(jsonObj.at("accelBiasMSec2"));
  params.gyroBiasRadSec = eigenVectorFromJson<float, 3>(jsonObj.at("gyroBiasRadSec"));
  params.accelNonorth = eigenMatrixFromJson<float, 3, 3>(jsonObj.at("accelNonorth"));
  params.gyroNonorth = eigenMatrixFromJson<float, 3, 3>(jsonObj.at("gyroNonorth"));
  params.gyroGSensitivityRadSecPerMSec2 =
      eigenMatrixFromJson<float, 3, 3>(jsonObj.at("gyroGSensitivityRadSecPerMSec2"));
  params.dtReferenceAccelSec = jsonObj.at("dtReferenceAccelSec").get<float>();
  params.dtReferenceGyroSec = jsonObj.at("dtReferenceGyroSec").get<float>();
  params.T_Imu_BodyImu = se3FromJson<float>(jsonObj.at("T_Imu_BodyImu"));
  params.nominalSamplingPeriodSec = jsonObj.at("nominalSamplingPeriodSec").get<float>();
  params.accelSaturationThresholdMSec2 = jsonObj.at("accelSaturationThresholdMSec2").get<float>();
  params.gyroSaturationThresholdRadSec = jsonObj.at("gyroSaturationThresholdRadSec").get<float>();
  return params;
}
} // namespace

// Function definitions for OnlineCalibState
json onlineCalibStateToJson(const OnlineCalibState& state) {
  json jsonObj;
  jsonObj["camParameters"] = json::array();
  for (const auto& camParam : state.camParameters) {
    jsonObj["camParameters"].push_back(projectionModelParametersToJson(camParam));
  }
  jsonObj["T_Cam_BodyImu"] = json::array();
  for (const auto& T : state.T_Cam_BodyImu) {
    jsonObj["T_Cam_BodyImu"].push_back(se3ToJson(T));
  }
  jsonObj["dt_Ref_Cam"] = state.dt_Ref_Cam;
  jsonObj["imuModelParameters"] = json::array();
  for (const auto& imuParam : state.imuModelParameters) {
    jsonObj["imuModelParameters"].push_back(imuMeasurementModelParametersToJson(imuParam));
  }
  return jsonObj;
}

std::string onlineCalibStateToJsonStr(const data_provider::OnlineCalibState& onlineCalibState) {
  return onlineCalibStateToJson(onlineCalibState).dump();
}

std::optional<OnlineCalibState> onlineCalibStateFromJson(const json& jsonObj) {
  try {
    OnlineCalibState state;
    for (const auto& camParamJson : jsonObj.at("camParameters")) {
      state.camParameters.push_back(projectionModelParametersFromJson(camParamJson));
    }
    for (const auto& TJson : jsonObj.at("T_Cam_BodyImu")) {
      state.T_Cam_BodyImu.push_back(se3FromJson<float>(TJson));
    }
    state.dt_Ref_Cam = jsonObj.at("dt_Ref_Cam").get<std::vector<int64_t>>();
    for (const auto& imuParamJson : jsonObj.at("imuModelParameters")) {
      state.imuModelParameters.push_back(imuMeasurementModelParametersFromJson(imuParamJson));
    }
    return state;
  } catch (const std::exception& e) {
    // Handle parsing error, e.g., log the error
    fmt::print("ERROR: json parsing has failed for onlineCalibStateFromJson: {}", e.what());
    return std::nullopt;
  }
}

std::optional<data_provider::OnlineCalibState> onlineCalibStateFromJsonStr(
    const std::string& onlineCalibJsonStr) {
  json jsonObj = nlohmann::json::parse(onlineCalibJsonStr);
  return onlineCalibStateFromJson(jsonObj);
}
} // namespace projectaria::tools::json
