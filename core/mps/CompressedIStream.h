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

#pragma once

#include <boost/iostreams/filtering_streambuf.hpp>
#include <fstream>
#include <string>

namespace projectaria::tools::mps {

enum class StreamCompressionMode {
  NONE,
  GZIP,
};

using boost_filtering_inbuf = boost::iostreams::filtering_streambuf<boost::iostreams::input>;

class CompressedIStream : public std::istream {
 public:
  explicit CompressedIStream(
      const std::string& path,
      StreamCompressionMode compression = StreamCompressionMode::GZIP);

 private:
  std::ifstream backingIfstream_; // destruct AFTER inbuf_
  boost_filtering_inbuf inbuf_; // destruct BEFORE backingIfstream_
};
} // namespace projectaria::tools::mps
