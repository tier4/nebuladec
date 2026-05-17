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
#include <optional>  // for std::optional return type of resolve()
#include <regex>
#include <string>
#include <vector>

namespace nebuladec
{

namespace detail
{

/// Single token from a parsed template string. Either a literal chunk
/// (`is_placeholder == false`) or a placeholder name to be substituted
/// at `resolve()` time. Lives in `detail` because it is only used to
/// share the token type between `topic_mapping.cpp` and the private
/// `CompiledRule` cache below; user code must not depend on it.
struct TemplateToken
{
  bool is_placeholder{false};
  std::string value;
};

}  // namespace detail

/// @brief One entry in the user-supplied YAML mapping table.
///
/// `in_pattern`, `out_template`, and `frame_id_template` may reference
/// `<name>` placeholders. Each placeholder captures exactly one path
/// segment (`[^/]+`). Names repeated inside `in_pattern` are
/// backreferenced, so they must resolve to the same captured value for
/// the rule to match.
struct MappingRule
{
  std::string in_pattern;
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
  std::string frame_id;
};

/// @brief Compiled form of the user's config file.
///
/// Use `from_yaml_file` / `from_yaml_string` to build; they validate the
/// schema and throw `std::invalid_argument` on schema errors. `resolve()`
/// then matches a single packet topic against every rule. The first
/// matching rule wins unless two or more rules match -- that is a user
/// error (ambiguous config) and surfaces as `std::runtime_error`.
///
/// Thread-safety: a compiled instance is immutable after construction
/// and `resolve()` only reads the compiled rules + the input topic
/// string; it does not mutate any member. A single instance is safe
/// to share across threads, which lets orchestrators look up the
/// mapping for several packet topics concurrently from worker threads
/// without serialising on a mutex.
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
    /// Pre-tokenized `out_template` / `frame_id_template`. Tokenization
    /// only depends on the rule text, so parsing once at compile time
    /// avoids re-running it for every packet in `resolve()`.
    std::vector<detail::TemplateToken> out_tokens;
    std::vector<detail::TemplateToken> frame_tokens;
  };

  explicit TopicMapping(std::vector<CompiledRule> compiled);

  std::vector<CompiledRule> compiled_;
  std::vector<MappingRule> rules_;  // mirrors compiled_ for public access
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__TOPIC_MAPPING_HPP_
