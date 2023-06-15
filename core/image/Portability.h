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

// HOST / DEVICE Annotations
#if defined(__CUDACC__)

// For the NVCC specific function specifiers
#include <cuda_runtime.h>

#define PROJECTARIA_HOST_DEVICE __host__ __device__
#define PROJECTARIA_DEVICE_INLINE __device__ __forceinline__
#define PROJECTARIA_HOST_DEVICE_INLINE __host__ __device__ __forceinline__

#else // !defined(__CUDACC__)

#define PROJECTARIA_HOST_DEVICE
#define PROJECTARIA_DEVICE_INLINE inline
#define PROJECTARIA_HOST_DEVICE_INLINE inline

#endif // !defined(__CUDACC__)

#ifdef __CUDACC__

// Make sure that this build was configured with CUDA support
#ifndef PROJECTARIA_HAVE_CUDA
#error "To safely use CUDA, configure with PROJECTARIA_HAVE_CUDA=ON"
#endif

#endif

// There's no way of crashing the C++ code in .cu files and kernel code besides this
// CUDA can't call fflush(), but we need to to ensure the assert text gets printed
// Thus fill the buffer up here
#define PROJECTARIA_CUDASAFE_ASSERT(x, ...)   \
  do {                                        \
    if (!(x)) {                               \
      printf("Assertion failed: " #x "\n");   \
      const int* arc_assert_null_ptr = 0;     \
      printf("%i\n", arc_assert_null_ptr[0]); \
    }                                         \
  } while (false)
