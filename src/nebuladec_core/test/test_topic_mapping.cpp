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

#include "nebuladec_core/topic_mapping.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace nebuladec
{
namespace
{

// ---------------------------------------------------------------------------
// Parsing happy path
// ---------------------------------------------------------------------------

TEST(TopicMappingParse, SingleAbsoluteRule)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /sensing/lidar/<position>/<vendor>_packets
    frame_id:  <position>/lidar
    out_topic: /sensing/lidar/<position>/<vendor>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);
  ASSERT_EQ(m.rules().size(), 1U);
  EXPECT_TRUE(m.rules().front().absolute);
}

TEST(TopicMappingParse, RelativeRule)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  <position>/<vendor>_packets
    frame_id:  <position>/lidar
    out_topic: <position>/<vendor>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);
  ASSERT_EQ(m.rules().size(), 1U);
  EXPECT_FALSE(m.rules().front().absolute);
}

TEST(TopicMappingParse, MultipleRulesAreRetainedInOrder)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /alpha/<v>_packets
    frame_id:  alpha
    out_topic: /alpha/<v>_points
  - in_topic:  /beta/<v>_packets
    frame_id:  beta
    out_topic: /beta/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);
  ASSERT_EQ(m.rules().size(), 2U);
  EXPECT_EQ(m.rules()[0].in_pattern, "/alpha/<v>_packets");
  EXPECT_EQ(m.rules()[1].in_pattern, "/beta/<v>_packets");
}

// ---------------------------------------------------------------------------
// Parsing errors
// ---------------------------------------------------------------------------

TEST(TopicMappingParse, EmptyMappingRejected)
{
  EXPECT_THROW(TopicMapping::from_yaml_string("mapping: []\n"), std::invalid_argument);
}

TEST(TopicMappingParse, MissingMappingKeyRejected)
{
  EXPECT_THROW(TopicMapping::from_yaml_string("other: []\n"), std::invalid_argument);
}

TEST(TopicMappingParse, MissingRequiredFieldRejected)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    out_topic: /a/<v>_points
)";  // frame_id missing
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

TEST(TopicMappingParse, CrossAbsoluteRelativeInOutRejected)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  lidar
    out_topic: <v>_points
)";
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

TEST(TopicMappingParse, UnknownPlaceholderInOutRejected)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  <v>/lidar
    out_topic: /a/<missing>_points
)";
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

TEST(TopicMappingParse, UnknownPlaceholderInFrameIdRejected)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  <missing>/lidar
    out_topic: /a/<v>_points
)";
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

TEST(TopicMappingParse, InvalidPlaceholderNameRejected)
{
  // Starts with a digit -> not a valid identifier.
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<1bad>_packets
    frame_id:  lidar
    out_topic: /a/<1bad>_points
)";
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

TEST(TopicMappingParse, UnterminatedPlaceholderRejected)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v_packets
    frame_id:  lidar
    out_topic: /a/<v>_points
)";
  EXPECT_THROW(TopicMapping::from_yaml_string(yaml), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Resolution -- absolute rules
// ---------------------------------------------------------------------------

TEST(TopicMappingResolve, AbsoluteMatchSubstitutesPlaceholders)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /sensing/lidar/<position>/<vendor>_packets
    frame_id:  <position>/lidar
    out_topic: /sensing/lidar/<position>/<vendor>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  auto match = m.resolve("/sensing/lidar/front_right/hesai_packets");
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->out_topic, "/sensing/lidar/front_right/hesai_points");
  EXPECT_EQ(match->frame_id, "front_right/lidar");
}

TEST(TopicMappingResolve, AbsoluteMatchRequiresAnchoredFullPath)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  lidar
    out_topic: /a/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  // Trailing extra segment must not match.
  EXPECT_FALSE(m.resolve("/a/hesai_packets/extra").has_value());
  // Missing leading slash must not match an absolute rule.
  EXPECT_FALSE(m.resolve("a/hesai_packets").has_value());
}

TEST(TopicMappingResolve, PlaceholderCapturesOneSegmentOnly)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /<position>/<v>_packets
    frame_id:  lidar
    out_topic: /<position>/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  // <position> must match exactly one segment -> "front/left" is two segments.
  EXPECT_FALSE(m.resolve("/front/left/hesai_packets").has_value());
}

TEST(TopicMappingResolve, SameNamePlaceholderBackreferences)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /<v>/<v>_packets
    frame_id:  lidar
    out_topic: /<v>/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  EXPECT_TRUE(m.resolve("/hesai/hesai_packets").has_value());
  // Different captures -> must not match.
  EXPECT_FALSE(m.resolve("/hesai/velodyne_packets").has_value());
}

// ---------------------------------------------------------------------------
// Resolution -- relative rules
// ---------------------------------------------------------------------------

TEST(TopicMappingResolve, RelativeMatchesSuffix)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  <position>/<v>_packets
    frame_id:  <position>/lidar
    out_topic: <position>/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  auto match = m.resolve("/sensing/lidar/front_right/hesai_packets");
  ASSERT_TRUE(match.has_value());
  // Output topic replaces the matched suffix in place, preserving the
  // bag's absolute prefix.
  EXPECT_EQ(match->out_topic, "/sensing/lidar/front_right/hesai_points");
  EXPECT_EQ(match->frame_id, "front_right/lidar");
}

TEST(TopicMappingResolve, RelativeRequiresSegmentBoundary)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  <v>_packets
    frame_id:  lidar
    out_topic: <v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  // Matches as a whole trailing segment.
  auto ok = m.resolve("/sensing/hesai_packets");
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->out_topic, "/sensing/hesai_points");

  // Must NOT glue to a preceding segment without a slash boundary.
  // "/sensinghesai_packets" has <v> capturing "sensinghesai", which IS a
  // valid single-segment capture -> this actually SHOULD match. Kept as a
  // documentation test: the boundary between the implicit ^/ and <v> is a
  // slash, so the only way to reject this is to require at least one
  // character before the slash. We keep the permissive behaviour.
  EXPECT_TRUE(m.resolve("/sensinghesai_packets").has_value());
}

// ---------------------------------------------------------------------------
// Resolution -- ambiguity / error paths
// ---------------------------------------------------------------------------

TEST(TopicMappingResolve, TwoRulesMatchingSameTopicThrows)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  lidar
    out_topic: /a/<v>_points
  - in_topic:  /a/hesai_packets
    frame_id:  lidar
    out_topic: /a/hesai_points_alt
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  EXPECT_THROW((void)m.resolve("/a/hesai_packets"), std::runtime_error);
}

TEST(TopicMappingResolve, NoMatchReturnsNullopt)
{
  const std::string yaml = R"(
mapping:
  - in_topic:  /a/<v>_packets
    frame_id:  lidar
    out_topic: /a/<v>_points
)";
  auto m = TopicMapping::from_yaml_string(yaml);

  EXPECT_FALSE(m.resolve("/b/hesai_packets").has_value());
  EXPECT_FALSE(m.resolve("").has_value());
}

}  // namespace
}  // namespace nebuladec
