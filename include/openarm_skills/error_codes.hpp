// Copyright 2026 User
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
//
// Single source of truth for skill-layer error codes.  These integers are
// echoed verbatim into the JSON envelope produced by openarm_api so the
// upper computer (or LLM agent) can react programmatically.  Keep this file
// in sync with `openarm_api/openarm_api/schemas/error_codes.json`.
#pragma once

namespace openarm_skills::err
{

constexpr int OK = 0;

// 1xxx -- motion / control
constexpr int PLAN_FAILED = 1001;             // MoveIt could not plan / Cartesian fraction too low
constexpr int ACTION_TIMEOUT = 1002;          // step exceeded step_timeout_s
constexpr int GRIP_NOT_HELD = 1003;           // finger position indicates empty grasp
constexpr int EXECUTE_FAILED = 1004;          // controller reported execution failure

// 2xxx -- perception
constexpr int CAMERA_NO_CLOUD = 2001;
constexpr int CAMERA_NO_TARGET = 2002;
constexpr int CAMERA_OUT_OF_RANGE = 2003;     // pose outside workspace bounds
constexpr int PERCEPTION_TIMEOUT = 2004;

// 3xxx -- input / protocol
constexpr int BAD_REQUEST = 3001;             // schema / argument validation failed
constexpr int UNSUPPORTED_CMD = 3002;

// 9xxx -- lifecycle
constexpr int STOPPED_BY_USER = 9001;
constexpr int INTERNAL_ERROR = 9002;

}  // namespace openarm_skills::err
