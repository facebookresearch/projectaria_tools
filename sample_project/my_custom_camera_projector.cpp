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

#include "lib/MyCustomCameraModel.h"

#include <iostream>

int main() {
  using namespace std;

  // define camera parameters
  double k1 = -0.25;
  double k2 = 0.05;
  double w = 2048;
  double h = 1536;

  MyCustomCameraModel cameraModel(k1, k2);
  Eigen::Vector4d intrinsincs(w / 2, h / 2, w / 2, h / 2);

  cout << "Sample project executable compiled correctly and ran!" << endl;
  cout << "created custom camera model with radial distortion with: " << endl;
  cout << "distortion coefficients [k1, k2]: "
       << cameraModel.getDistortionCoefficients().transpose() << endl;
  cout << "instrinsics [fx, fy, cx, cy]: " << intrinsincs.transpose() << endl;

  Eigen::Vector3d pointInCam(0.3, 0.3, 3);
  cout << "projecting point in camera frame with coordinates [x, y, z] : " << pointInCam.transpose()
       << endl;

  Eigen::Vector2d pixels = cameraModel.projectWithRadialDistortion(pointInCam, intrinsincs);
  cout << "point projected to pixel coordinates [u, v]: " << pixels.transpose() << endl;

  return 0;
}
