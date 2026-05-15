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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nebuladec
{

namespace
{

using Token = detail::TemplateToken;

bool is_identifier_start(char c)
{
  return std::isalpha(static_cast<unsigned char>(c)) != 0 || c == '_';
}

bool is_identifier_body(char c)
{
  return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

/// Parse a template string (topic pattern, out template, frame_id
/// template) into a flat sequence of (literal, placeholder) tokens.
///
/// Throws std::invalid_argument for:
///   * unterminated placeholder (missing `>`)
///   * empty placeholder name
///   * invalid placeholder name (not [A-Za-z_][A-Za-z0-9_]*)
std::vector<Token> tokenize_template(const std::string & text, const std::string & field_name)
{
  std::vector<Token> tokens;
  std::string literal;
  for (std::size_t i = 0; i < text.size();) {
    if (text[i] != '<') {
      literal.push_back(text[i]);
      ++i;
      continue;
    }
    // Flush any accumulated literal before emitting a placeholder token.
    if (!literal.empty()) {
      tokens.push_back({false, std::move(literal)});
      literal.clear();
    }
    const auto close = text.find('>', i + 1);
    if (close == std::string::npos) {
      throw std::invalid_argument(
        "topic_mapping: unterminated placeholder in " + field_name + ": '" + text + "'");
    }
    const std::string name = text.substr(i + 1, close - i - 1);
    if (name.empty()) {
      throw std::invalid_argument(
        "topic_mapping: empty placeholder name in " + field_name + ": '" + text + "'");
    }
    if (!is_identifier_start(name.front())) {
      throw std::invalid_argument(
        "topic_mapping: placeholder name must start with a letter or underscore in " + field_name +
        ": '<" + name + ">'");
    }
    for (char c : name) {
      if (!is_identifier_body(c)) {
        throw std::invalid_argument(
          "topic_mapping: invalid character in placeholder name in " + field_name + ": '<" + name +
          ">'");
      }
    }
    tokens.push_back({true, name});
    i = close + 1;
  }
  if (!literal.empty()) {
    tokens.push_back({false, std::move(literal)});
  }
  return tokens;
}

/// Escape a literal string so it can be dropped into a std::regex pattern.
std::string escape_regex_literal(const std::string & text)
{
  std::string out;
  out.reserve(text.size() * 2);
  for (char c : text) {
    switch (c) {
      case '.':
      case '^':
      case '$':
      case '|':
      case '(':
      case ')':
      case '[':
      case ']':
      case '{':
      case '}':
      case '+':
      case '*':
      case '?':
      case '\\':
      case '/':
        out.push_back('\\');
        break;
      default:
        break;
    }
    out.push_back(c);
  }
  return out;
}

/// Compile `tokens` (from `in_pattern`) into a std::regex.
///   * `absolute == true`:  pattern is anchored with `^<...>$`.
///   * `absolute == false`: pattern is anchored with `^.*/<...>$` so the
///     whole template must match a suffix ending on a path boundary. An
///     unboundaried match (e.g. "/sensinghesai_packets" for pattern
///     "<v>_packets") still matches because the leading `/` and the
///     single-segment placeholder `[^/]+` collapse to "sensinghesai".
///
/// `placeholder_names` is populated in order of first appearance. Repeats
/// are encoded as backreferences so they must resolve to the same capture.
std::regex compile_in_regex(
  const std::vector<Token> & tokens, bool absolute, std::vector<std::string> & placeholder_names,
  std::size_t & placeholder_group_offset)
{
  std::unordered_map<std::string, std::size_t> name_to_group;
  std::ostringstream pattern;

  pattern << '^';
  if (absolute) {
    // Leading '/' is part of the literal in tokens already (the first
    // token is a literal that begins with '/').
    placeholder_group_offset = 1;
  } else {
    // Relative pattern: reserve group 1 for the absolute prefix captured
    // off the incoming topic so `resolve()` can splice it back onto the
    // rewritten suffix. The prefix ends on a slash boundary; an empty
    // prefix (no absolute prefix at all) is also accepted.
    pattern << "(.*/)?";
    placeholder_group_offset = 2;
  }

  for (const auto & tok : tokens) {
    if (!tok.is_placeholder) {
      pattern << escape_regex_literal(tok.value);
      continue;
    }
    auto it = name_to_group.find(tok.value);
    if (it == name_to_group.end()) {
      const std::size_t group_index = placeholder_names.size() + placeholder_group_offset;
      name_to_group.emplace(tok.value, group_index);
      placeholder_names.push_back(tok.value);
      pattern << "([^/]+)";
    } else {
      // Backreference: enforce same captured value for repeated names.
      pattern << "\\" << it->second;
    }
  }
  pattern << '$';
  return std::regex(pattern.str());
}

/// Turn `tokens` into a fill-in-the-blanks template using already-captured
/// placeholder values. Throws std::invalid_argument if `tokens` references
/// a name not in `known_names` (checked by the caller before calling).
std::string expand_template(
  const std::vector<Token> & tokens, const std::unordered_map<std::string, std::string> & captures)
{
  std::string out;
  for (const auto & tok : tokens) {
    if (!tok.is_placeholder) {
      out += tok.value;
      continue;
    }
    auto it = captures.find(tok.value);
    if (it == captures.end()) {
      // Should be unreachable by construction (validated at parse time).
      throw std::invalid_argument("topic_mapping: unknown placeholder at expansion: " + tok.value);
    }
    out += it->second;
  }
  return out;
}

bool starts_with_slash(const std::string & s)
{
  return !s.empty() && s.front() == '/';
}

/// Check that every placeholder in `tokens` was already captured by the
/// in-pattern (names in `known_names`). Throws std::invalid_argument if
/// any placeholder is unknown.
void ensure_placeholders_known(
  const std::vector<Token> & tokens, const std::unordered_set<std::string> & known_names,
  const std::string & field_name)
{
  for (const auto & tok : tokens) {
    if (tok.is_placeholder && !known_names.count(tok.value)) {
      throw std::invalid_argument(
        "topic_mapping: placeholder <" + tok.value + "> in " + field_name +
        " is not declared in in_topic");
    }
  }
}

std::string required_scalar(
  const YAML::Node & node, const std::string & key, std::size_t rule_index)
{
  if (!node[key] || !node[key].IsScalar()) {
    throw std::invalid_argument(
      "topic_mapping: rule " + std::to_string(rule_index) + " missing required field '" + key +
      "'");
  }
  const auto value = node[key].as<std::string>();
  if (value.empty()) {
    throw std::invalid_argument(
      "topic_mapping: rule " + std::to_string(rule_index) + " has empty '" + key + "'");
  }
  return value;
}

}  // namespace

TopicMapping::TopicMapping(std::vector<CompiledRule> compiled) : compiled_(std::move(compiled))
{
  rules_.reserve(compiled_.size());
  for (const auto & cr : compiled_) {
    rules_.push_back(cr.rule);
  }
}

TopicMapping TopicMapping::from_yaml_file(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("topic_mapping: failed to parse YAML '" + path + "': " + e.what());
  }
  try {
    return from_yaml_string(YAML::Dump(root));
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument(std::string{"topic_mapping ("} + path + "): " + e.what());
  }
}

TopicMapping TopicMapping::from_yaml_string(const std::string & yaml)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception & e) {
    throw std::invalid_argument(std::string{"topic_mapping: invalid YAML: "} + e.what());
  }
  if (!root["mapping"] || !root["mapping"].IsSequence()) {
    throw std::invalid_argument("topic_mapping: missing top-level 'mapping' sequence");
  }
  const auto mapping = root["mapping"];
  if (mapping.size() == 0) {
    throw std::invalid_argument("topic_mapping: 'mapping' must not be empty");
  }

  std::vector<CompiledRule> compiled;
  compiled.reserve(mapping.size());

  for (std::size_t i = 0; i < mapping.size(); ++i) {
    const auto node = mapping[i];
    if (!node.IsMap()) {
      throw std::invalid_argument(
        "topic_mapping: rule " + std::to_string(i) + " must be a mapping");
    }
    MappingRule rule;
    rule.in_pattern = required_scalar(node, "in_topic", i);
    rule.out_template = required_scalar(node, "out_topic", i);
    rule.frame_id_template = required_scalar(node, "frame_id", i);
    rule.absolute = starts_with_slash(rule.in_pattern);

    if (rule.absolute != starts_with_slash(rule.out_template)) {
      throw std::invalid_argument(
        "topic_mapping: rule " + std::to_string(i) +
        ": in_topic and out_topic must both be absolute or both relative");
    }

    auto in_tokens = tokenize_template(rule.in_pattern, "in_topic");
    auto out_tokens = tokenize_template(rule.out_template, "out_topic");
    auto frame_tokens = tokenize_template(rule.frame_id_template, "frame_id");

    CompiledRule cr;
    cr.rule = rule;
    cr.regex =
      compile_in_regex(in_tokens, rule.absolute, cr.placeholder_names, cr.placeholder_group_offset);

    std::unordered_set<std::string> known_names(
      cr.placeholder_names.begin(), cr.placeholder_names.end());
    ensure_placeholders_known(out_tokens, known_names, "out_topic");
    ensure_placeholders_known(frame_tokens, known_names, "frame_id");

    // Cache the tokenized output / frame_id templates so resolve() does
    // not re-parse them per packet on the convert() hot path.
    cr.out_tokens = std::move(out_tokens);
    cr.frame_tokens = std::move(frame_tokens);

    compiled.push_back(std::move(cr));
  }

  return TopicMapping{std::move(compiled)};
}

std::optional<MappingMatch> TopicMapping::resolve(const std::string & in_topic) const
{
  std::optional<MappingMatch> match;
  std::size_t match_rule_index = 0;

  for (std::size_t i = 0; i < compiled_.size(); ++i) {
    const auto & cr = compiled_[i];
    std::smatch sm;
    if (!std::regex_match(in_topic, sm, cr.regex)) {
      continue;
    }
    if (match.has_value()) {
      throw std::runtime_error(
        "topic_mapping: topic '" + in_topic + "' matches multiple rules (rules " +
        std::to_string(match_rule_index) + " and " + std::to_string(i) + ")");
    }

    // Build a name->captured-value map from the regex submatches. For
    // relative rules, placeholders start at group 2 (group 1 is the
    // absolute prefix of the incoming topic).
    std::unordered_map<std::string, std::string> captures;
    for (std::size_t p = 0; p < cr.placeholder_names.size(); ++p) {
      captures.emplace(cr.placeholder_names[p], sm[p + cr.placeholder_group_offset].str());
    }

    MappingMatch m;
    m.rule_index = i;
    m.out_topic = expand_template(cr.out_tokens, captures);
    m.frame_id = expand_template(cr.frame_tokens, captures);

    // For relative rules, the absolute prefix of the incoming topic
    // lives in group 1. Splice it back onto the rewritten suffix so,
    // e.g., /sensing/lidar/front/hesai_packets with rule <v>_packets ->
    // /sensing/lidar/front/hesai_points.
    if (!cr.rule.absolute) {
      const std::string prefix = sm[1].matched ? sm[1].str() : std::string{};
      m.out_topic = prefix + m.out_topic;
    }

    match = std::move(m);
    match_rule_index = i;
  }

  return match;
}

}  // namespace nebuladec
