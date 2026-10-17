---
description: Run pre-commit hooks (clang-format, codespell, etc.) over the staged + changed files, then optionally clang-tidy.
argument-hint: "[--all | <file paths>]"
allowed-tools: ["Bash"]
---

Lint changed files in `~/gz-sim`.

1. If `$ARGUMENTS` is empty, run only over changed-vs-main:
   `pre-commit run --from-ref origin/main --to-ref HEAD`
2. If `$ARGUMENTS == "--all"`, run everything:
   `pre-commit run --all-files`
3. Otherwise treat `$ARGUMENTS` as a space-separated file list:
   `pre-commit run --files $ARGUMENTS`
4. If clang-tidy is wanted (ask the user), run it against the compile
   database in `build/compile_commands.json`:
   `clang-tidy -p build $ARGUMENTS`
5. Report any auto-fixes that pre-commit applied — the user needs to
   stage them.
6. Do **not** add `--no-verify` to anything; if a hook fails, fix the
   root cause.
