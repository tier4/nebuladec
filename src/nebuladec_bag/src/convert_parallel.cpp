// Copyright 2026 TIER IV, Inc.
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

// The parallel convert pipeline now lives entirely in `bag_io.cpp`:
// it is a straight producer-consumer graph (reader -> per-topic
// in-queues -> worker pool -> single shared out-queue -> writer) and
// needs no out-of-line helpers. The associated header defines header-
// only `AbortFlag` and `BoundedQueue<T>` primitives. This translation
// unit is retained so `CMakeLists.txt`'s source list stays stable and
// to leave a hook for future helpers without forcing a build-system
// edit.

#include "convert_parallel.hpp"
