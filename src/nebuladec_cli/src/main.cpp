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
//   nebuladec convert <input> -o <output> -c <yaml> [--dry-run]

#include <nebula_core_common/nebula_common.hpp>
#include <nebuladec_bag/bag_io.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>
#include <nebuladec_core/topic_mapping.hpp>
#include <third_party/CLI11.hpp>
#include <third_party/indicators.hpp>
#include <third_party/tabulate.hpp>

#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
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
  } else {
    return "<unknown>";
  }
  return os.str();
}

int print_inspect_only(const std::string & input_path)
{
  // Dry-run without a config: fall back to a vendor/model report of every
  // packet topic in the bag. Useful for users who want to see what's in a
  // bag before writing a mapping config.
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
  bool have_output{false};
  bool have_config{false};
  bool dry_run{false};
  // 0 means "auto" -- the library picks min(hardware_concurrency, K)
  // where K is the number of decoded LiDAR topics. Mutually exclusive
  // with --sequential at the CLI level (CLI11 `excludes`).
  std::size_t workers{0};
  bool sequential{false};
  // Forcefully disable the indicators progress bar. Even on a TTY the
  // bar is suppressed when this is true. Independent of --dry-run /
  // workers / sequential.
  bool no_progress{false};
  // Raw flag values; parsed by `parse_mcap_*` below into the typed
  // McapWriteOptions struct the library consumes. Empty = unset.
  std::string mcap_compression_raw;
  std::string mcap_chunk_size_raw;
};

/// Parse `--mcap-compression VALUE` where VALUE is one of:
///   none
///   lz4 | lz4:fastest|fast|default|slow|slowest
///   zstd | zstd:fastest|fast|default|slow|slowest
/// Sets both `compression` and `compression_level` fields. Throws
/// `CLI::ValidationError` on a bad value so CLI11 turns it into an
/// exit-64 usage error.
void parse_mcap_compression(const std::string & raw, nebuladec::bag::McapWriteOptions & out)
{
  if (raw.empty()) {
    return;
  }
  const auto colon = raw.find(':');
  const std::string kind = raw.substr(0, colon);
  const std::string level = colon == std::string::npos ? std::string{} : raw.substr(colon + 1);

  if (kind == "none") {
    if (!level.empty()) {
      throw CLI::ValidationError("--mcap-compression: 'none' takes no level suffix");
    }
    out.compression = nebuladec::bag::McapCompression::kNone;
    out.compression_level = nebuladec::bag::McapCompressionLevel::kAuto;
    return;
  }
  if (kind == "lz4") {
    out.compression = nebuladec::bag::McapCompression::kLz4;
  } else if (kind == "zstd") {
    out.compression = nebuladec::bag::McapCompression::kZstd;
  } else {
    throw CLI::ValidationError(
      "--mcap-compression: expected 'none', 'lz4[:level]', or 'zstd[:level]' (got '" + raw + "')");
  }

  if (level.empty() || level == "default") {
    out.compression_level = nebuladec::bag::McapCompressionLevel::kDefault;
  } else if (level == "fastest") {
    out.compression_level = nebuladec::bag::McapCompressionLevel::kFastest;
  } else if (level == "fast") {
    out.compression_level = nebuladec::bag::McapCompressionLevel::kFast;
  } else if (level == "slow") {
    out.compression_level = nebuladec::bag::McapCompressionLevel::kSlow;
  } else if (level == "slowest") {
    out.compression_level = nebuladec::bag::McapCompressionLevel::kSlowest;
  } else {
    throw CLI::ValidationError(
      "--mcap-compression: expected level fastest|fast|default|slow|slowest (got '" + level + "')");
  }
}

/// Parse `--mcap-chunk-size BYTES` where BYTES is a positive integer
/// with an optional K/KB/M/MB/G/GB suffix (case-insensitive, binary
/// SI: K=1024, M=1024*1024, G=1024*1024*1024). Throws
/// `CLI::ValidationError` on bad input.
std::uint64_t parse_chunk_size(const std::string & raw)
{
  if (raw.empty()) {
    return 0;
  }
  std::size_t pos = 0;
  std::uint64_t value = 0;
  try {
    value = std::stoull(raw, &pos);
  } catch (const std::exception &) {
    throw CLI::ValidationError("--mcap-chunk-size: not a number (got '" + raw + "')");
  }
  if (value == 0U) {
    throw CLI::ValidationError("--mcap-chunk-size: must be positive (got '" + raw + "')");
  }
  std::string suffix = raw.substr(pos);
  std::transform(suffix.begin(), suffix.end(), suffix.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::uint64_t multiplier = 1;
  if (suffix.empty() || suffix == "b") {
    multiplier = 1;
  } else if (suffix == "k" || suffix == "kb" || suffix == "kib") {
    multiplier = 1024ULL;
  } else if (suffix == "m" || suffix == "mb" || suffix == "mib") {
    multiplier = 1024ULL * 1024ULL;
  } else if (suffix == "g" || suffix == "gb" || suffix == "gib") {
    multiplier = 1024ULL * 1024ULL * 1024ULL;
  } else {
    throw CLI::ValidationError(
      "--mcap-chunk-size: unknown suffix '" + suffix + "' (expected K/M/G or none)");
  }
  return value * multiplier;
}

/// RAII wrapper around the indicators overall-decode progress bar.
///
/// Construction is conditional: when stdout is not a TTY, when the
/// caller asked for `--no-progress`, or when the bag has nothing to
/// decode, `make()` returns an empty callable so the bag library skips
/// the entire progress path. When active, the destructor restores the
/// console cursor and prints a trailing newline so the next line
/// (result table / error) starts on a fresh row even if the user
/// Ctrl-C'd mid-decode.
class ProgressBarGuard
{
public:
  static std::function<void(const nebuladec::bag::ProgressEvent &)> make(bool no_progress)
  {
    if (no_progress || isatty(fileno(stdout)) == 0) {
      return {};
    }
    auto guard = std::make_shared<ProgressBarGuard>();
    return [guard](const nebuladec::bag::ProgressEvent & ev) { guard->update(ev); };
  }

  ProgressBarGuard()
  : bar_(
      std::make_unique<indicators::BlockProgressBar>(
        indicators::option::BarWidth{40}, indicators::option::Start{"["},
        indicators::option::End{"]"}, indicators::option::PrefixText{"Decoding "},
        indicators::option::ForegroundColor{indicators::Color::cyan},
        indicators::option::FontStyles{
          std::vector<indicators::FontStyle>{indicators::FontStyle::bold}})),
    start_(std::chrono::steady_clock::now())
  {
    // indicators' built-in ShowElapsedTime / ShowRemainingTime render
    // mm:ss only; we want millisecond precision so we drive the elapsed
    // string ourselves via PostfixText below.
    indicators::show_console_cursor(false);
  }

  ~ProgressBarGuard()
  {
    if (bar_ && !bar_->is_completed()) {
      bar_->mark_as_completed();
    }
    indicators::show_console_cursor(true);
    // The bar uses CR (\r) writes; landing on a fresh line keeps any
    // subsequent stdout (result table) from overprinting the bar row.
    std::cout << '\n';
  }

  ProgressBarGuard(const ProgressBarGuard &) = delete;
  ProgressBarGuard & operator=(const ProgressBarGuard &) = delete;
  ProgressBarGuard(ProgressBarGuard &&) = delete;
  ProgressBarGuard & operator=(ProgressBarGuard &&) = delete;

  void update(const nebuladec::bag::ProgressEvent & ev)
  {
    if (!bar_) {
      return;
    }
    // total == 0 means "no decoded topics"; the bag library won't fire
    // this callback in that case, but guard against future shifts.
    if (ev.messages_total == 0U) {
      return;
    }
    const auto pct =
      (100.0 * static_cast<double>(ev.messages_done)) / static_cast<double>(ev.messages_total);

    // Format elapsed time at millisecond precision (e.g. "12.345s").
    // The bag library throttles `on_progress` to ~50 ms, so the bar
    // can update visibly faster than 1 s — millisecond digits make
    // short conversions readable instead of stuck on "00s".
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start_)
                              .count();
    const auto whole_s = elapsed_ms / 1000;
    const auto frac_ms = elapsed_ms % 1000;

    std::ostringstream postfix;
    postfix << whole_s << '.' << std::setw(3) << std::setfill('0') << frac_ms << "s | "
            << ev.messages_done << " / " << ev.messages_total << " messages";
    bar_->set_option(indicators::option::PostfixText{postfix.str()});
    bar_->set_progress(static_cast<float>(pct));
  }

private:
  std::unique_ptr<indicators::BlockProgressBar> bar_;
  std::chrono::steady_clock::time_point start_;
};

int print_dry_run(const std::vector<nebuladec::bag::ConvertPlanEntry> & entries)
{
  if (entries.empty()) {
    std::cout << "no packet topics found in bag\n";
    return k_exit_ok;
  }
  tabulate::Table table;
  // Column order mirrors the post-convert table so users can compare
  // dry-run plans against actual conversions at a glance.
  table.add_row({"input", "output", "frame_id", "vendor", "model", "packets"});
  std::size_t ok_count = 0;
  std::size_t skipped_count = 0;
  std::size_t error_count = 0;
  for (const auto & e : entries) {
    const bool no_messages = (e.status == "skipped" && e.message == "no messages");
    const std::string vendor_cell = no_messages ? std::string{"-"} : identity_vendor(e.identity);
    const std::string model_cell = no_messages ? std::string{"-"} : identity_model(e.identity);
    const std::string out = e.status == "ok" ? e.out_topic : std::string{"-"};
    const std::string frame = e.status == "ok" ? e.frame_id : std::string{"-"};
    table.add_row({e.in_topic, out, frame, vendor_cell, model_cell, std::to_string(e.packets)});
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

int run_convert(const ConvertCliOptions & opts)
{
  try {
    if (opts.dry_run && !opts.have_config) {
      return print_inspect_only(opts.input_path);
    }

    nebuladec::TopicMapping mapping;
    try {
      mapping = nebuladec::TopicMapping::from_yaml_file(opts.config_path);
    } catch (const std::exception & e) {
      std::cerr << "failed to load config: " << e.what() << "\n";
      return k_exit_usage;
    }

    if (opts.dry_run) {
      const auto entries = nebuladec::bag::plan_convert(opts.input_path, mapping);
      return print_dry_run(entries);
    }

    nebuladec::bag::ConvertOptions options;
    options.input_path = opts.input_path;
    options.output_path = opts.output_path;
    options.mapping = std::move(mapping);
    options.sequential = opts.sequential;
    options.workers = opts.workers;
    // The progress callback keeps a shared_ptr to the bar guard, so
    // the guard outlives convert() and cleans up the terminal even if
    // convert() throws.
    options.on_progress = ProgressBarGuard::make(opts.no_progress);
    // MCAP writer tuning. Only honoured when the output bag is MCAP
    // (the output path's extension drives the output plugin for
    // bare-file outputs); the library warns and ignores them on
    // sqlite3 output.
    parse_mcap_compression(opts.mcap_compression_raw, options.mcap);
    options.mcap.chunk_size_bytes = parse_chunk_size(opts.mcap_chunk_size_raw);
    const auto result = nebuladec::bag::convert(options);

    if (result.topics.empty()) {
      std::cerr << "no packet topic was decoded; output bag contains passthrough data only\n";
      // Passthrough-only output is still a valid result, not a failure.
    }

    tabulate::Table table;
    table.add_row({"input", "output", "frame_id", "vendor", "model", "packets", "clouds"});
    for (const auto & t : result.topics) {
      table.add_row(
        {t.in_topic, t.out_topic, t.frame_id, identity_vendor(t.identity),
         identity_model(t.identity), std::to_string(t.packets), std::to_string(t.clouds_written)});
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
  CLI::App app{"nebuladec -- decode Nebula packet bags to PointCloud2"};
  app.require_subcommand(1);
  app.set_help_flag("-h,--help", "Print this help message and exit");

  auto * convert = app.add_subcommand(
    "convert",
    "Decode every packet topic that matches the YAML mapping into "
    "PointCloud2, and preserve every other topic from the input bag verbatim. "
    "Topics matching no rule, or whose vendor/model is not supported, are "
    "copied through unchanged.");
  convert->set_help_flag("-h,--help", "Print this help message and exit");

  ConvertCliOptions opts;
  convert->add_option("input", opts.input_path, "Input bag path")->required();
  auto * out_opt = convert->add_option(
    "-o,--output", opts.output_path, "Output bag path (required unless --dry-run is set).");
  auto * cfg_opt = convert->add_option(
    "-c,--config", opts.config_path,
    "Mapping config (YAML). Required for actual conversion; optional with "
    "--dry-run (in which case dry-run reduces to a vendor/model report of "
    "the input bag).");
  convert->add_flag(
    "--dry-run", opts.dry_run,
    "Report what would be decoded without writing any bag. With --config, "
    "prints the full resolution plan; without it, prints vendor/model for "
    "each packet topic.");

  // Parallel pipeline knobs (see `nebuladec_bag` README "Performance"
  // section). The 3-stage pipeline is the default; these flags only
  // override that default.
  auto * workers_opt = convert->add_option(
    "-j,--workers", opts.workers,
    "Decoder worker pool size for the parallel pipeline. 0 (default) "
    "= auto = min(hardware_concurrency, K) where K is the number of "
    "decoded LiDAR topics. When the requested value is below K it is "
    "snapped down to the largest divisor of K so each worker owns the "
    "same number of topics. Ignored under --dry-run.");
  workers_opt->check(CLI::NonNegativeNumber);
  auto * sequential_flag = convert->add_flag(
    "--sequential", opts.sequential,
    "Force the legacy single-threaded code path. Mutually exclusive "
    "with --workers; useful as a fall-back for byte-for-byte regression "
    "comparison or on hosts with fewer than 3 hardware threads. "
    "Ignored under --dry-run.");
  workers_opt->excludes(sequential_flag);

  convert->add_flag(
    "--no-progress", opts.no_progress,
    "Suppress the decode progress bar. The bar is shown by default on a "
    "TTY when the bag contains at least one decoded LiDAR topic, and "
    "automatically hidden when stdout is not a TTY (CI logs, pipes). "
    "Ignored under --dry-run.");

  // MCAP writer tuning. The output storage plugin is driven by the
  // output path's extension (.mcap / .db3) for bare-file outputs, so
  // these flags only take effect when the output bag is MCAP. The
  // library logs a warning and ignores them on sqlite3 output.
  convert->add_option(
    "--mcap-compression", opts.mcap_compression_raw,
    "MCAP output compression. Format: 'none' | 'lz4[:LEVEL]' | "
    "'zstd[:LEVEL]' where LEVEL is fastest|fast|default|slow|slowest. "
    "Examples: --mcap-compression none, --mcap-compression zstd:fast, "
    "--mcap-compression lz4. Default: writer plugin default (zstd / "
    "default level). Ignored under --dry-run and when the output bag "
    "is not .mcap.");
  convert->add_option(
    "--mcap-chunk-size", opts.mcap_chunk_size_raw,
    "MCAP output chunk size in bytes. Accepts an integer with optional "
    "K/M/G suffix (binary SI: K=1024). Examples: --mcap-chunk-size 4M, "
    "--mcap-chunk-size 16777216. Larger chunks reduce the number of "
    "compression invocations on writer-bound workloads at the cost of "
    "extra memory per chunk. Default: writer plugin default (~768 KiB). "
    "Ignored under --dry-run and when the output bag is not .mcap.");

  // Flag matrix (mirrors prior hand-rolled behavior):
  //   --dry-run + no --config  -> inspect-style report
  //   --dry-run + --config     -> full resolution plan
  //   (no --dry-run)           -> requires both --output and --config
  convert->callback([&]() {
    opts.have_output = out_opt->count() > 0;
    opts.have_config = cfg_opt->count() > 0;
    if (!opts.dry_run && (!opts.have_output || !opts.have_config)) {
      throw CLI::ValidationError("--output and --config are required without --dry-run");
    }
    // --workers / --sequential touch the convert pipeline; dry-run
    // never enters that pipeline, so warn rather than silently accept
    // a misleading invocation.
    if (opts.dry_run && (workers_opt->count() > 0 || sequential_flag->count() > 0)) {
      std::cerr << "warning: --workers / --sequential are ignored with --dry-run\n";
    }
  });

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    const int code = app.exit(e);
    // Preserve historical exit codes: 0 for help/version, 64 for any usage
    // error. CLI11 returns its own non-zero codes (e.g. 105 for RequiredError)
    // which would surprise existing callers.
    return code == 0 ? k_exit_ok : k_exit_usage;
  }

  if (convert->parsed()) {
    return run_convert(opts);
  }
  return k_exit_usage;
}
