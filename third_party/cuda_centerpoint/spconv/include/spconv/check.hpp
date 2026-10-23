/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
 
#ifndef __SPCONV_CHECK_HPP__
#define __SPCONV_CHECK_HPP__

#include <assert.h>
#include <cuda_runtime.h>
#include <string>
#include <spconv/engine.hpp>

namespace spconv {

#define check_cuda_api(call) spconv::check_cuda_api_(call, #call, __LINE__, __FILE__)
#define check_cuda_kernel(...)                                                            \
  do {                                                                              \
    __VA_ARGS__;                                                                    \
    spconv::check_cuda_api_(cudaPeekAtLastError(), #__VA_ARGS__, __LINE__, __FILE__); \
  } while (0)

#define spconv_assertf(cond, fmt, ...)                                                                 \
  do {                                                                                          \
    if (!(cond)) {                                                                              \
      spconv::logger_output(__FILE__, __LINE__, spconv::LoggerLevel::Error, "Assert failed 💀. %s in file %s:%d, message: " fmt, #cond, __FILE__, \
              __LINE__, __VA_ARGS__);                                                           \
    }                                                                                           \
  } while (false)

#define spconv_asserts(cond, s)                                                                      \
  do {                                                                                        \
    if (!(cond)) {                                                                            \
      spconv::logger_output(__FILE__, __LINE__, spconv::LoggerLevel::Error, "Assert failed 💀. %s in file %s:%d, message: " s, #cond, __FILE__, \
              __LINE__);                                                                      \
    }                                                                                         \
  } while (false)

#define spconv_assert(cond)                                                                     \
  do {                                                                                   \
    if (!(cond)) {                                                                       \
      spconv::logger_output(__FILE__, __LINE__, spconv::LoggerLevel::Error, "Assert failed 💀. %s in file %s:%d", #cond, __FILE__, __LINE__); \
    }                                                                                    \
  } while (false)

#define spconv_logf(level, fmt, ...)  spconv::logger_output(__FILE__, __LINE__, level, fmt, __VA_ARGS__);
#define spconv_log(level, msg)        spconv::logger_output(__FILE__, __LINE__, level, "%s", msg);

Exported std::string format(const char *fmt, ...);
Exported bool check_cuda_api_(cudaError_t e, const char *call, int line, const char *file);

};  // namespace spconv

#endif  // #ifndef __SPCONV_CHECK_HPP__