---
description: Run the gz-sim CTest suite (optionally filtered by regex) and report pass / fail counts.
argument-hint: "[ctest -R regex, e.g. INTEGRATION_diff_drive]"
allowed-tools: ["Bash"]
---

Run the gz-sim test suite from `~/gz-sim`.

1. Make sure `build/` exists. If not, tell the user to run `/gz-build` first.
2. If `$ARGUMENTS` is empty:
   `ctest --test-dir build --output-on-failure -j"$(nproc)"`
   else:
   `ctest --test-dir build -R "$ARGUMENTS" --output-on-failure`
3. After the run, report a one-line summary like
   `"148 passed, 0 failed, 2 skipped (12m13s)"`.
4. On failures, list the first 3 failing test names and offer to drop into
   the gtest binary directly (see the `gz-test` skill).
5. Never claim "tests pass" without showing the count and `ctest`'s
   exit code. If anything was skipped or filtered out, mention it.
