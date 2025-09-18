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

#include <cstdint>
#include <cstring>

#include <ostream>
#include <string>
#include <utility>

namespace projectaria::tools::data_provider {

/** Type-safe struct for a UUID. */
template <typename Purpose>
struct alignas(8) Uuid {
  /// UUID standard is 16 bytes == 128 bits. See: https://www.ietf.org/rfc/rfc4122.txt
  static constexpr const int kStaticSize = 16;

  /// Attempts to parse a UUID from the given string.
  /// Returns the parsed value (if successful), and a boolean indicating the success. Returns an
  /// invalid UUID (all zero) upon failure.
  static std::pair<Uuid, bool> createFromString(const std::string& str);

  static Uuid createFromIds(std::uint64_t id1, std::uint64_t id2);

  /// This function is added to easy conversion between vision uuid and a lot uuid
  static Uuid createFromBytes(const std::uint8_t bytes[kStaticSize]);

  /// Returns an invalid UUID (all zeros).
  // Note: Consider making default constructor private once we move away from cereal.
  static Uuid createInvalidUuid();

  /// Returns the canonical textual representation.
  [[nodiscard]] std::string toString() const;

  /// Returns a new UUID object of the same value, but for another purpose
  template <typename NewPurpose>
  Uuid<NewPurpose> repurposedClone() const;

  /// Returns false if the UUID is invalid (all zero).
  [[nodiscard]] bool isValid() const;
  [[nodiscard]] bool isInvalid() const {
    return !isValid();
  }

  std::uint8_t value[kStaticSize] = {0};

}; // struct Uuid

template <typename Purpose, class Archive>
void serialize(Archive& ar, Uuid<Purpose>& uuid) {
  ar(uuid.value);
}

static_assert(
    sizeof(Uuid<struct TestType>) == Uuid<struct TestType>::kStaticSize,
    "UUID size is not 16 bytes");
static_assert(alignof(Uuid<struct TestType>) == 8, "UUID is not aligned to 8 bytes");

template <typename Purpose>
bool operator==(Uuid<Purpose> a, Uuid<Purpose> b);

template <typename Purpose>
bool operator!=(Uuid<Purpose> a, Uuid<Purpose> b);

template <typename Purpose>
bool operator<(Uuid<Purpose> a, Uuid<Purpose> b);

//
// Inline implementations.
//

namespace detail {

inline bool parseDigit(std::uint8_t digit, std::uint8_t& dst) {
  if (digit >= '0' && digit <= '9') {
    dst = digit - '0';
    return true;
  } else if (digit >= 'a' && digit <= 'f') {
    dst = digit - 'a' + 10;
    return true;
  } else if (digit >= 'A' && digit <= 'F') {
    dst = digit - 'A' + 10;
    return true;
  } else {
    return false;
  }
}

inline bool parseByte(const std::string& str, int pos, std::uint8_t& dst) {
  std::uint8_t hi = 0;
  std::uint8_t lo = 0;
  if (!parseDigit(str[pos + 0], hi)) {
    return false;
  }
  if (!parseDigit(str[pos + 1], lo)) {
    return false;
  }
  dst = (hi << 4) | lo;
  return true;
}

inline void appendDigit(std::uint8_t byte, std::string& str) {
  if (byte < 10) {
    str += static_cast<uint8_t>('0' + byte);
  } else {
    str += static_cast<uint8_t>('a' + (byte - 10));
  }
}

inline void appendByte(std::uint8_t byte, std::string& str) {
  appendDigit((byte >> 4) & 0x0f, str);
  appendDigit((byte >> 0) & 0x0f, str);
}

} // namespace detail

template <typename Purpose>
std::pair<Uuid<Purpose>, bool> Uuid<Purpose>::createFromString(const std::string& str) {
  if (str.size() != 36) {
    return std::make_pair(Uuid<Purpose>(), false);
  }
  if (str[8] != '-' || str[13] != '-' || str[18] != '-' || str[23] != '-') {
    return std::make_pair(Uuid<Purpose>(), false);
  }
  Uuid<Purpose> output;
  for (int i = 0; i < 4; ++i) {
    if (!detail::parseByte(str, (2 * i) + 0, output.value[i])) {
      return std::make_pair(Uuid<Purpose>(), false);
    }
  }
  for (int i = 4; i < 6; ++i) {
    if (!detail::parseByte(str, (2 * i) + 1, output.value[i])) {
      return std::make_pair(Uuid<Purpose>(), false);
    }
  }
  for (int i = 6; i < 8; ++i) {
    if (!detail::parseByte(str, (2 * i) + 2, output.value[i])) {
      return std::make_pair(Uuid<Purpose>(), false);
    }
  }
  for (int i = 8; i < 10; ++i) {
    if (!detail::parseByte(str, (2 * i) + 3, output.value[i])) {
      return std::make_pair(Uuid<Purpose>(), false);
    }
  }
  for (int i = 10; i < 16; ++i) {
    if (!detail::parseByte(str, (2 * i) + 4, output.value[i])) {
      return std::make_pair(Uuid<Purpose>(), false);
    }
  }

  return std::make_pair(output, true);
}

template <typename Purpose>
Uuid<Purpose> Uuid<Purpose>::createFromIds(std::uint64_t id1, std::uint64_t id2) {
  Uuid<Purpose> output;
  auto* val64Ptr = reinterpret_cast<std::uint64_t*>(output.value);
  val64Ptr[0] = id1;
  val64Ptr[1] = id2;
  return output;
}

template <typename Purpose>
Uuid<Purpose> Uuid<Purpose>::createFromBytes(const std::uint8_t bytes[kStaticSize]) {
  Uuid<Purpose> output;
  auto* val64Ptr = reinterpret_cast<std::uint64_t*>(output.value);
  const auto* inPtr = (const uint64_t*)(bytes);
  val64Ptr[0] = inPtr[0];
  val64Ptr[1] = inPtr[1];
  return output;
}

template <typename Purpose>
Uuid<Purpose> Uuid<Purpose>::createInvalidUuid() {
  return Uuid<Purpose>();
}

template <typename Purpose>
std::string Uuid<Purpose>::toString() const {
  std::string output;
  output.reserve(36);
  for (int i = 0; i < 4; ++i) {
    detail::appendByte(value[i], output);
  }
  output += '-';
  for (int i = 4; i < 6; ++i) {
    detail::appendByte(value[i], output);
  }
  output += '-';
  for (int i = 6; i < 8; ++i) {
    detail::appendByte(value[i], output);
  }
  output += '-';
  for (int i = 8; i < 10; ++i) {
    detail::appendByte(value[i], output);
  }
  output += '-';
  for (int i = 10; i < 16; ++i) {
    detail::appendByte(value[i], output);
  }
  return output;
}

template <typename Purpose>
template <typename NewPurpose>
Uuid<NewPurpose> Uuid<Purpose>::repurposedClone() const {
  return *reinterpret_cast<const Uuid<NewPurpose>*>(this);
}

template <typename Purpose>
inline bool Uuid<Purpose>::isValid() const {
  const auto* ptr = reinterpret_cast<const std::uint64_t*>(this->value);
  return ptr[0] != 0 || ptr[1] != 0;
}

template <typename Purpose>
inline bool operator==(Uuid<Purpose> a, Uuid<Purpose> b) {
  // Using explicit cast to 64-bit numbers yields slightly better assembly code than relying on the
  // compiler to optimize std::memcmp (tested with godbolt.org for both x86_64 and arm64).
  const auto* aPtr = reinterpret_cast<const std::uint64_t*>(a.value);
  const auto* bPtr = reinterpret_cast<const std::uint64_t*>(b.value);
  return (aPtr[0] == bPtr[0] && aPtr[1] == bPtr[1]);
}

template <typename Purpose>
inline bool operator!=(Uuid<Purpose> a, Uuid<Purpose> b) {
  return !operator==(a, b);
}

template <typename Purpose>
inline bool operator<(Uuid<Purpose> a, Uuid<Purpose> b) {
  // The compiler won't optimize calls to std::memcmp, presumably because the relevant systems are
  // not big endian. The actual order doesn't matter as long as it's consistent, hence we simply
  // cast to 64-bit numbers.
  // godbolt.org shows that this results in quite few instructions on both x86_64 and arm64.
  const auto* aPtr = reinterpret_cast<const std::uint64_t*>(a.value);
  const auto* bPtr = reinterpret_cast<const std::uint64_t*>(b.value);
  if (aPtr[0] == bPtr[0]) {
    return aPtr[1] < bPtr[1];
  }
  return aPtr[0] < bPtr[0];
}

template <typename Purpose>
inline std::ostream& operator<<(std::ostream& out, const Uuid<Purpose>& uuid) {
  out << uuid.toString();
  return out;
}

} // namespace projectaria::tools::data_provider

namespace std {
/// Also provide a specialized hash function.
/// Assuming we're dealing with random UUIDs simply XOR the two 64-bit numbers.
/// (use the same logic with 32-bit numbers on 32-bit systems).
template <typename Purpose>
struct hash<projectaria::tools::data_provider::Uuid<Purpose>> {
  std::size_t operator()(
      const projectaria::tools::data_provider::Uuid<Purpose>& uuid) const noexcept {
    static_assert(
        sizeof(std::size_t) == 4 || sizeof(std::size_t) == 8,
        "std::size_t is neither 8 nor 4 bytes wide.");

    if (sizeof(std::size_t) == 8) {
      const auto* ptr = reinterpret_cast<const std::uint64_t*>(uuid.value);
      return ptr[0] ^ ptr[1];
    } else {
      const auto* ptr = reinterpret_cast<const std::uint32_t*>(uuid.value);
      return ptr[0] ^ ptr[1] ^ ptr[2] ^ ptr[3];
    }
  }
};
} // namespace std

// Custom fmt-style formatting
#if __has_include(<fmt/format.h>)

#include <fmt/format.h>

template <typename Purpose>
struct fmt::formatter<projectaria::tools::data_provider::Uuid<Purpose>> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) const -> decltype(ctx.begin()) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const projectaria::tools::data_provider::Uuid<Purpose>& uuid, FormatContext& ctx)
      const {
    return fmt::format_to(ctx.out(), "{}", uuid.toString());
  }
};

#endif // __has_include(<fmt/format.h>)
