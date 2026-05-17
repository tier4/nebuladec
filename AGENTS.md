# AGENTS.md

Guidelines for AI agents contributing to this repository.

## 1. Code & Documentation

- Always write source code and documentation in English.
- Whenever source code is modified, the corresponding documentation
  (e.g. `README.md`, command-line help text, in-source comments) MUST
  be updated in the same change. Source and documentation must always
  describe the same behavior — leaving them out of sync is not
  acceptable, even temporarily.

## 2. Attribution

- Do NOT include AI agent signatures (e.g. `Co-Authored-By: <agent name> ...`)
  in any generated code, commit messages, pull request descriptions,
  documentation, or other output.

## 3. Commit Messages & Branch Names

- Follow the [Conventional Commits](https://www.conventionalcommits.org/)
  specification for every commit message. Use one of the standard types
  (`feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`, `ci`,
  `style`, `build`, `revert`) with an optional scope, e.g.
  `feat(bag): add multi-topic inspect`.
- Use the same type as the branch prefix when creating a branch for a
  pull request (e.g. `feat/multi-topic-inspect`, `fix/hesai-sop-order`).
  Do not use tool- or author-specific prefixes such as `claude/*`.

## 4. Pre-commit Hooks

- Every pre-commit hook MUST end in the `Passed` state. Never bypass
  hooks (e.g. `--no-verify`) and never disable, broaden, or globally
  ignore a rule to push work through — fix the underlying issue by
  refactoring the source code so all hooks pass cleanly.
- If a hook reports what is clearly a false positive, you MAY suppress
  it inline at the narrowest possible scope (e.g. a single line or
  symbol) with an explanatory comment. Inline suppression is the only
  acceptable form of bypass.

## 5. GitHub Actions / CI

- Be mindful of the GitHub Actions workflows configured in this
  repository; ensure changes do not cause them to fail.
- When investigating workflow failures, use the `gh` command
  (e.g. `gh run view`, `gh run view --log-failed`) to retrieve and read
  the actual logs. Base bug fixes on evidence from those logs, not on
  assumptions.

## 6. Coding Conventions

- Unless given explicit instructions to the contrary, analyze the
  existing codebase's naming conventions and design patterns and
  follow them when writing new code. Stay consistent with the
  surrounding code rather than introducing a new style.
- Prefer many small functions with a single, well-defined
  responsibility over large functions that perform multiple unrelated
  steps. Each function should do one thing, have a clear scope, and
  be named after that one responsibility; extract helpers when a
  function starts mixing concerns.
- Avoid reinventing the wheel. Before implementing non-trivial
  functionality, check whether a well-maintained open-source library
  already solves the problem; if a suitable OSS alternative exists,
  proactively propose it (with a brief note on maintenance status,
  license, and fit) instead of writing a custom implementation.
- For C++ code, you MUST follow RAII for any type that owns a dynamic
  resource (heap memory, file descriptors, sockets, locks, GPU
  handles, etc.): acquire the resource in the constructor and release
  it in the destructor, and make ownership semantics explicit via
  smart pointers (`std::unique_ptr`, `std::shared_ptr`) or equivalent
  owning wrappers. Code must be free of memory leaks, double-free,
  use-after-free, dangling pointers, and forgotten releases — prefer
  the rule of zero, and only fall back to the rule of five with a
  clear justification. When the ECC (Everything Claude Code) plugin
  is installed, additionally verify the code against the broader
  checklist documented by the `/ecc:cpp-coding-standards` skill
  (which subsumes these RAII rules and extends them with the full C++
  Core Guidelines) and fix any violations in the affected locations
  before considering the work complete.

## 7. Remote Repository Operations

- Always obtain explicit developer approval before making any changes
  to the remote repository — pushing commits, creating/closing pull
  requests or issues, commenting on PRs, and so on.
- Do not push directly to the `main` branch. Always open a pull request
  first.
- Write PR descriptions that are comprehensive and detailed, yet
  concise: cover the problem, the solution, and the test plan without
  unnecessary verbosity.
