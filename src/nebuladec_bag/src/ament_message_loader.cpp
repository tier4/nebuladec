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

#include "ament_message_loader.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace nebuladec::bag
{

namespace
{

namespace fs = std::filesystem;

// Mirrors the regex used by `rosbag2_storage_mcap`'s
// MessageDefinitionCache so dependency parsing yields byte-identical
// output across the two implementations.
const std::regex g_package_typename_regex{
  R"(^([a-zA-Z][a-zA-Z0-9_]*)/((?:msg|srv)/)?([a-zA-Z][a-zA-Z0-9_]*)$)"};
const std::regex g_msg_field_type_regex{R"((?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+)"};

const std::unordered_set<std::string> g_primitive_types{
  "bool",   "byte",  "char",   "float32", "float64", "int8",   "uint8",  "int16",
  "uint16", "int32", "uint32", "int64",   "uint64",  "string", "wstring"};

struct ParsedTypename
{
  std::string package;
  std::string short_name;  // without the "msg/" infix
};

[[nodiscard]] std::optional<ParsedTypename> parse_typename(std::string_view type_name)
{
  std::smatch m;
  const std::string copy{type_name};
  if (!std::regex_match(copy, m, g_package_typename_regex)) {
    return std::nullopt;
  }
  return ParsedTypename{m[1].str(), m[3].str()};
}

[[nodiscard]] std::optional<std::string> read_file(const fs::path & path)
{
  std::ifstream in{path};
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream oss;
  oss << in.rdbuf();
  return oss.str();
}

[[nodiscard]] std::optional<std::string> load_raw_msg_text(const ParsedTypename & pt)
{
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory(pt.package);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return std::nullopt;
  }
  // Two filename conventions exist in the wild: <pkg>/msg/<Name>.msg
  // (the standard layout) and a flat <pkg>/<Name>.msg (legacy / a few
  // third-party packages). Try both before declaring failure.
  for (const auto & candidate :
       {fs::path{share} / "msg" / (pt.short_name + ".msg"),
        fs::path{share} / (pt.short_name + ".msg")}) {
    if (auto contents = read_file(candidate); contents) {
      return contents;
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::set<std::string> parse_dependencies(
  const std::string & text, const std::string & package_context)
{
  std::set<std::string> deps;
  for (std::sregex_iterator it{text.begin(), text.end(), g_msg_field_type_regex};
       it != std::sregex_iterator{}; ++it) {
    std::string type = (*it)[1].str();
    if (g_primitive_types.count(type) > 0) {
      continue;
    }
    if (type.find('/') == std::string::npos) {
      deps.insert(package_context + "/" + std::move(type));
    } else {
      deps.insert(std::move(type));
    }
  }
  return deps;
}

/// Recursively concatenate the .msg text for `target` plus every
/// non-primitive dependency it references. Visited types are tracked in
/// `seen` so cyclic / repeated references are walked exactly once.
[[nodiscard]] bool append_definition(
  const std::string & target_resource_name, std::ostringstream & out,
  std::unordered_set<std::string> & seen, bool first)
{
  if (!seen.insert(target_resource_name).second) {
    return true;  // already emitted
  }
  const auto parsed = parse_typename(target_resource_name);
  if (!parsed) {
    return false;
  }
  const auto text = load_raw_msg_text(*parsed);
  if (!text) {
    return false;
  }
  if (!first) {
    // The 80-char `=` separator + `MSG: pkg/Type` header is the
    // ros2msg concatenation contract defined at
    // https://mcap.dev/specification/appendix.html#ros2msg-data-format.
    out << "================================================================================\n";
    out << "MSG: " << parsed->package << "/" << parsed->short_name << "\n";
  }
  out << *text;
  if (!text->empty() && text->back() != '\n') {
    out << '\n';
  }

  for (const auto & dep : parse_dependencies(*text, parsed->package)) {
    if (!append_definition(dep, out, seen, /*first=*/false)) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::optional<MessageDefinition> load_message_definition_from_ament(std::string_view type_name)
{
  const auto parsed = parse_typename(type_name);
  if (!parsed) {
    return std::nullopt;
  }
  std::ostringstream out;
  std::unordered_set<std::string> seen;
  if (!append_definition(std::string{type_name}, out, seen, /*first=*/true)) {
    return std::nullopt;
  }
  MessageDefinition def;
  def.type_name = std::string{type_name};
  def.encoding = "ros2msg";
  def.text = out.str();
  return def;
}

}  // namespace nebuladec::bag
