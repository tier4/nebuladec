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
//   nebuladec convert <input> -o <output> [--packets-topic <name>]
//                             [--info-topic <name>] [--output-topic <name>]
//                             [--frame-id <name>]

#include <nebula_core_common/nebula_common.hpp>
#include <nebuladec_bag/bag_io.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>
#include <third_party/tabulate.hpp>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr int k_exit_ok = 0;
constexpr int k_exit_usage = 64;
constexpr int k_exit_runtime = 70;

int print_usage(std::ostream & out)
{
  out << "usage: nebuladec <subcommand> [options]\n"
         "\n"
         "subcommands:\n"
         "  inspect <path>\n"
         "      Report vendor/model for every packet topic found in the\n"
         "      bag (LiDAR and Continental radar). Reads only the first\n"
         "      message per topic.\n"
         "\n"
         "  convert <input> -o <output> [options]\n"
         "      Decode packets from one LiDAR topic and write a sibling\n"
         "      PointCloud2 bag. Pass --packets-topic when the bag has\n"
         "      more than one packet-stream topic.\n"
         "\n"
         "convert options:\n"
         "  -o, --output <path>          Output bag path (required).\n"
         "      --output-topic <name>    PointCloud2 topic to write (default "
         "/nebuladec/pointcloud).\n"
         "      --packets-topic <name>   Pick the input packet topic explicitly.\n"
         "      --info-topic <name>      Override Robosense info topic auto-detection.\n"
         "      --frame-id <name>        Frame id on written PointCloud2 (default lidar).\n"
         "\n"
         "The output bag uses the same storage plugin (mcap/sqlite3) and\n"
         "layout (file vs metadata directory) as the input.\n";
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

int cmd_inspect(const std::vector<std::string> & argv)
{
  if (argv.size() != 1) {
    return print_usage(std::cerr);
  }
  try {
    const auto summary = nebuladec::bag::inspect(argv[0]);
    if (summary.topics.empty()) {
      std::cout << "no Nebula packet topics found in bag\n";
      return k_exit_ok;
    }
    std::cout << "topics discovered: " << summary.topics.size() << "\n";

    tabulate::Table table;
    table.add_row({"topic", "vendor", "model"});
    for (const auto & t : summary.topics) {
      table.add_row({t.topic, identity_vendor(t.identity), identity_model(t.identity)});
    }
    table[0].format().font_style({tabulate::FontStyle::bold});
    std::cout << table << "\n";
    return k_exit_ok;
  } catch (const std::exception & e) {
    std::cerr << "inspect failed: " << e.what() << "\n";
    return k_exit_runtime;
  }
}

int cmd_convert(const std::vector<std::string> & argv)
{
  nebuladec::bag::ConvertOptions options;
  bool have_output = false;
  bool have_input = false;

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
        return print_usage(std::cerr);
      }
      options.output_path = *v;
      have_output = true;
    } else if (arg == "--output-topic") {
      auto v = next(arg);
      if (!v) {
        return print_usage(std::cerr);
      }
      options.output_topic = *v;
    } else if (arg == "--packets-topic") {
      auto v = next(arg);
      if (!v) {
        return print_usage(std::cerr);
      }
      options.packets_topic = *v;
    } else if (arg == "--info-topic") {
      auto v = next(arg);
      if (!v) {
        return print_usage(std::cerr);
      }
      options.info_topic = *v;
    } else if (arg == "--frame-id") {
      auto v = next(arg);
      if (!v) {
        return print_usage(std::cerr);
      }
      options.frame_id = *v;
    } else if (arg == "-h" || arg == "--help") {
      print_usage(std::cout);
      return k_exit_ok;
    } else if (arg.rfind("-", 0) == 0) {
      std::cerr << "unknown option: " << arg << "\n";
      return print_usage(std::cerr);
    } else if (!have_input) {
      options.input_path = arg;
      have_input = true;
    } else {
      std::cerr << "unexpected positional argument: " << arg << "\n";
      return print_usage(std::cerr);
    }
  }

  if (!have_input || !have_output) {
    return print_usage(std::cerr);
  }

  try {
    const auto result = nebuladec::bag::convert(options);
    std::cerr << "identity         : " << format_identity(result.identity) << "\n";
    std::cerr << "data packets     : " << result.data_packets << "\n";
    std::cerr << "info packets     : " << result.info_packets << "\n";
    std::cerr << "clouds written   : " << result.clouds_written << "\n";
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

  if (subcmd == "inspect") {
    return cmd_inspect(rest);
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
