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

/*
This macro will change all XR_LOG to std::cout.
Since setGlobalLogLevel cannot disable the XR_LOG, changing to std::cout will disable all logging
when --disable-logging is added
*/
#include <iostream>

#undef XR_LOGI
#define XR_LOGI(...)                                                                       \
  std::cout << "[" << DEFAULT_LOG_CHANNEL << "][INFO]" << ": " << fmt::format(__VA_ARGS__) \
            << std::endl;

#undef XR_LOGE
#define XR_LOGE(...)                                                                        \
  std::cout << "[" << DEFAULT_LOG_CHANNEL << "][ERROR]" << ": " << fmt::format(__VA_ARGS__) \
            << std::endl;

#undef XR_LOGW
#define XR_LOGW(...)                                                                          \
  std::cout << "[" << DEFAULT_LOG_CHANNEL << "][WARNING]" << ": " << fmt::format(__VA_ARGS__) \
            << std::endl;
