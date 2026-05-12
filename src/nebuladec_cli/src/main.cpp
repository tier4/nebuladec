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
//
// `nebuladec` command-line entry point.
//
//   nebuladec inspect <path>
//   nebuladec convert <input> -o <output> -c <yaml> [--dry-run]

#include <nebula_core_common/nebula_common.hpp>
#include <nebuladec_bag/bag_io.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>
#include <nebuladec_core/topic_mapping.hpp>
#include <third_party/tabulate.hpp>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr int k_exit_ok = 0;
constexpr int k_exit_usage = 64;
constexpr int k_exit_runtime = 70;

int print_usage(std::ostream & out)
{
  out << "usage: nebuladec convert <input> [options]\n"
         "\n"
         "  Decode every packet topic that matches the YAML mapping into\n"
         "  PointCloud2, and preserve every other topic from the input\n"
         "  bag verbatim. Topics matching no rule, or whose vendor/model\n"
         "  is not supported, are copied through unchanged.\n"
         "\n"
         "options:\n"
         "  -o, --output <path>   Output bag path (required unless\n"
         "                        --dry-run is set).\n"
         "  -c, --config <yaml>   Mapping config. Required for actual\n"
         "                        conversion; optional with --dry-run\n"
         "                        (in which case dry-run reduces to a\n"
         "                        vendor/model report of the input bag).\n"
         "  --dry-run             Report what would be decoded without\n"
         "                        writing any bag. With --config, prints\n"
         "                        the full resolution plan; without it,\n"
         "                        prints vendor/model for each packet\n"
         "                        topic.\n";
  return k_exit_usage;
}

std::string format_identity(const std::optional<nebuladec::Identity> & id)
{
  if (!id) {
    return "<unknown>";
  }
  std::ostringstream os;
  os << nebuladec::to_string(id->vendor) << "/";
  // Seyond carries its model in a separate enum (seyond_model) because
  // SensorModel::UNKNOWN stays set even after the sniffer identifies a
  // Seyond model. Fall through to that enum when the main one is blank.
  if (id->model != nebula::drivers::SensorModel::UNKNOWN) {
    os << id->model;  // nebula_common.hpp overloads operator<< on ostream
  } else if (id->seyond_model) {
    os << *id->seyond_model;
  } else {
    os << "unknown-model";
  }
  return os.str();
}

std::string identity_vendor(const std::optional<nebuladec::Identity> & id)
{
  return id ? nebuladec::to_string(id->vendor) : "<unknown>";
}

std::string identity_model(const std::optional<nebuladec::Identity> & id)
{
  if (!id) {
    return "<unknown>";
  }
  std::ostringstream os;
  if (id->model != nebula::drivers::SensorModel::UNKNOWN) {
    os << id->model;
  } else if (id->seyond_model) {
    os << *id->seyond_model;
  } else {
    return "<unknown>";
  }
  return os.str();
}

int print_inspect_only(const std::string & input_path)
{
  // Dry-run without a config: fall back to a vendor/model report of every
  // packet topic in the bag. Equivalent to the now-removed `inspect`
  // subcommand and useful for users who want to see what's in a bag
  // before writing a mapping config.
  const auto summary = nebuladec::bag::inspect(input_path);
  if (summary.topics.empty()) {
    std::cout << "no Nebula packet topics found in bag\n";
    return k_exit_ok;
  }

  tabulate::Table table;
  table.add_row({"topic", "vendor", "model"});
  std::size_t visible = 0;
  std::size_t hidden_empty = 0;
  for (const auto & t : summary.topics) {
    if (!t.has_messages) {
      ++hidden_empty;
      continue;
    }
    table.add_row({t.topic, identity_vendor(t.identity), identity_model(t.identity)});
    ++visible;
  }
  if (visible == 0) {
    std::cout << "no Nebula packet topics with messages in bag (" << hidden_empty
              << " empty topic(s) hidden)\n";
    return k_exit_ok;
  }
  std::cout << "topics discovered: " << visible;
  if (hidden_empty > 0) {
    std::cout << " (" << hidden_empty << " empty topic(s) hidden)";
  }
  std::cout << "\n";
  table[0].format().font_style({tabulate::FontStyle::bold});
  std::cout << table << "\n";
  return k_exit_ok;
}

struct ConvertCliOptions
{
  std::string input_path;
  std::string output_path;
  std::string config_path;
  bool have_input{false};
  bool have_output{false};
  bool have_config{false};
  bool dry_run{false};
};

std::optional<ConvertCliOptions> parse_convert_args(const std::vector<std::string> & argv)
{
  ConvertCliOptions opts;

  for (std::size_t i = 0; i < argv.size(); ++i) {
    const auto & arg = argv[i];
    auto next = [&](const std::string & name) -> std::optional<std::string> {
      if (i + 1 >= argv.size()) {
        std::cerr << "missing value for " << name << "\n";
        return std::nullopt;
      }
      return argv[++i];
    };

    if (arg == "-o" || arg == "--output") {
      auto v = next(arg);
      if (!v) {
        return std::nullopt;
      }
      opts.output_path = *v;
      opts.have_output = true;
    } else if (arg == "-c" || arg == "--config") {
      auto v = next(arg);
      if (!v) {
        return std::nullopt;
      }
      opts.config_path = *v;
      opts.have_config = true;
    } else if (arg == "--dry-run") {
      opts.dry_run = true;
    } else if (arg == "-h" || arg == "--help") {
      print_usage(std::cout);
      std::exit(k_exit_ok);
    } else if (arg.rfind("-", 0) == 0) {
      std::cerr << "unknown option: " << arg << "\n";
      return std::nullopt;
    } else if (!opts.have_input) {
      opts.input_path = arg;
      opts.have_input = true;
    } else {
      std::cerr << "unexpected positional argument: " << arg << "\n";
      return std::nullopt;
    }
  }
  return opts;
}

int print_dry_run(const std::vector<nebuladec::bag::ConvertPlanEntry> & entries)
{
  if (entries.empty()) {
    std::cout << "no packet topics found in bag\n";
    return k_exit_ok;
  }
  tabulate::Table table;
  table.add_row({"in_topic", "vendor", "model", "out_topic", "frame_id", "decodable"});
  std::size_t ok_count = 0;
  std::size_t skipped_count = 0;
  std::size_t error_count = 0;
  for (const auto & e : entries) {
    const bool no_messages = (e.status == "skipped" && e.message == "no messages");
    const std::string vendor_cell = no_messages ? std::string{"-"} : identity_vendor(e.identity);
    const std::string model_cell = no_messages ? std::string{"-"} : identity_model(e.identity);
    const std::string out = e.status == "ok" ? e.out_topic : std::string{"-"};
    const std::string frame = e.status == "ok" ? e.frame_id : std::string{"-"};
    // "decodable" collapses the three internal status values into a
    // yes/no answer for the user, with the original reason appended when
    // the answer is no. The resolved/skipped/errors summary line below
    // preserves the finer split for debugging ambiguous configs.
    std::string decodable_cell;
    if (e.status == "ok") {
      decodable_cell = "yes";
    } else {
      decodable_cell = "no";
      if (!e.message.empty()) {
        decodable_cell += ": " + e.message;
      }
    }
    table.add_row({e.in_topic, vendor_cell, model_cell, out, frame, decodable_cell});
    if (e.status == "ok") {
      ++ok_count;
    } else if (e.status == "skipped") {
      ++skipped_count;
    } else {
      ++error_count;
    }
  }
  table[0].format().font_style({tabulate::FontStyle::bold});
  std::cout << table << "\n";
  std::cout << "resolved: " << ok_count << "  skipped: " << skipped_count
            << "  errors: " << error_count << "\n";
  return error_count == 0 ? k_exit_ok : k_exit_runtime;
}

int cmd_convert(const std::vector<std::string> & argv)
{
  auto parsed = parse_convert_args(argv);
  if (!parsed) {
    return print_usage(std::cerr);
  }
  if (!parsed->have_input) {
    return print_usage(std::cerr);
  }
  // Flag matrix:
  //   --dry-run + no --config  -> inspect-style report (no mapping)
  //   --dry-run + --config     -> full resolution plan
  //   (no --dry-run) + -o + --config -> actual convert
  //   anything else            -> usage error
  if (!parsed->dry_run) {
    if (!parsed->have_output || !parsed->have_config) {
      return print_usage(std::cerr);
    }
  }

  try {
    if (parsed->dry_run && !parsed->have_config) {
      return print_inspect_only(parsed->input_path);
    }

    nebuladec::TopicMapping mapping;
    try {
      mapping = nebuladec::TopicMapping::from_yaml_file(parsed->config_path);
    } catch (const std::exception & e) {
      std::cerr << "failed to load config: " << e.what() << "\n";
      return k_exit_usage;
    }

    if (parsed->dry_run) {
      const auto entries = nebuladec::bag::plan_convert(parsed->input_path, mapping);
      return print_dry_run(entries);
    }

    nebuladec::bag::ConvertOptions options;
    options.input_path = parsed->input_path;
    options.output_path = parsed->output_path;
    options.mapping = std::move(mapping);
    const auto result = nebuladec::bag::convert(options);

    if (!result.passthrough_topics.empty()) {
      std::cerr << "preserved " << result.passthrough_topics.size()
                << " topic(s) verbatim via passthrough:\n";
      for (const auto & t : result.passthrough_topics) {
        std::cerr << "  " << t << "\n";
      }
    }

    if (result.topics.empty()) {
      std::cerr << "no packet topic was decoded; output bag contains passthrough data only\n";
      // Passthrough-only output is still a valid result, not a failure.
    }

    tabulate::Table table;
    table.add_row(
      {"in_topic", "out_topic", "frame_id", "identity", "data_pkts", "info_pkts", "clouds"});
    for (const auto & t : result.topics) {
      table.add_row(
        {t.in_topic, t.out_topic, t.frame_id, format_identity(t.identity),
         std::to_string(t.data_packets), std::to_string(t.info_packets),
         std::to_string(t.clouds_written)});
    }
    table[0].format().font_style({tabulate::FontStyle::bold});
    std::cout << table << "\n";
    return k_exit_ok;
  } catch (const std::exception & e) {
    std::cerr << "convert failed: " << e.what() << "\n";
    return k_exit_runtime;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    return print_usage(std::cerr);
  }

  const std::string subcmd = argv[1];
  std::vector<std::string> rest;
  rest.reserve(argc - 2);
  for (int i = 2; i < argc; ++i) {
    rest.emplace_back(argv[i]);
  }

  if (subcmd == "convert") {
    return cmd_convert(rest);
  }
  if (subcmd == "-h" || subcmd == "--help") {
    print_usage(std::cout);
    return k_exit_ok;
  }

  std::cerr << "unknown subcommand: " << subcmd << "\n";
  return print_usage(std::cerr);
}
