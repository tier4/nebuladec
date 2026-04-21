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

#ifndef NEBULADEC_CORE__TOPIC_MAPPING_HPP_
#define NEBULADEC_CORE__TOPIC_MAPPING_HPP_

#include <cstddef>
#include <optional>
#include <regex>
#include <string>
#include <vector>

namespace nebuladec
{

/// @brief One entry in the user-supplied YAML mapping table.
///
/// `in_pattern`, `out_template` (and, if present, `info_pattern` /
/// `frame_id_template`) may reference `<name>` placeholders. Each
/// placeholder captures exactly one path segment (`[^/]+`). Names repeated
/// inside `in_pattern` are backreferenced, so they must resolve to the
/// same captured value for the rule to match.
struct MappingRule
{
  std::string in_pattern;
  std::optional<std::string> info_pattern;
  std::string out_template;
  std::string frame_id_template;
  /// true when `in_pattern` starts with `/`. Every pattern in the same
  /// rule must agree on this (cross-absolute/relative mixing is rejected
  /// at parse time).
  bool absolute{false};
};

/// @brief Result of matching an incoming topic against a rule.
struct MappingMatch
{
  std::size_t rule_index{0};
  std::string out_topic;
  /// Empty when the rule has no `info_topic` field. Not a std::optional so
  /// downstream callers that just want to compare strings have one less
  /// null-check to write.
  std::string info_topic;
  std::string frame_id;
};

/// @brief Compiled form of the user's config file.
///
/// Use `from_yaml_file` / `from_yaml_string` to build; they validate the
/// schema and throw `std::invalid_argument` on schema errors. `resolve()`
/// then matches a single packet topic against every rule. The first
/// matching rule wins unless two or more rules match -- that is a user
/// error (ambiguous config) and surfaces as `std::runtime_error`.
class TopicMapping
{
public:
  /// Construct an empty mapping. `resolve()` returns `nullopt` for every
  /// input until rules are populated (use the factory functions below to
  /// build a non-empty mapping). Exists so that `TopicMapping` can be a
  /// plain member of `ConvertOptions`-style structs.
  TopicMapping() = default;

  /// Build from a file on disk. Throws `std::invalid_argument` on schema
  /// errors, `std::runtime_error` on I/O errors.
  static TopicMapping from_yaml_file(const std::string & path);

  /// Build from an in-memory YAML string. Throws `std::invalid_argument`
  /// on schema errors. Useful for tests.
  static TopicMapping from_yaml_string(const std::string & yaml);

  /// Match `in_topic` against every rule.
  /// Returns `std::nullopt` when no rule matches.
  /// Throws `std::runtime_error` when two or more rules match.
  std::optional<MappingMatch> resolve(const std::string & in_topic) const;

  const std::vector<MappingRule> & rules() const { return rules_; }

private:
  struct CompiledRule
  {
    MappingRule rule;
    std::regex regex;
    /// Placeholder names in order of first occurrence in `in_pattern`.
    std::vector<std::string> placeholder_names;
    /// Regex submatch index of the first placeholder. Equals 1 for
    /// absolute rules; equals 2 for relative rules (group 1 is reserved
    /// for the absolute prefix captured from the matched topic).
    std::size_t placeholder_group_offset{1};
  };

  explicit TopicMapping(std::vector<CompiledRule> compiled);

  std::vector<CompiledRule> compiled_;
  std::vector<MappingRule> rules_;  // mirrors compiled_ for public access
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__TOPIC_MAPPING_HPP_
