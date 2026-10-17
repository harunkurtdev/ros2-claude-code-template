---
description: Configure and build gz-sim with cmake + ninja (RelWithDebInfo) and report the outcome.
argument-hint: "[extra cmake args]"
allowed-tools: ["Bash"]
---

You are about to build gz-sim from `~/gz-sim`.

1. If `build/` does not yet exist, configure it:
   `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON $ARGUMENTS`
2. Build: `cmake --build build -j"$(nproc)"`
3. Print the last 30 lines of build output so the user can see warnings.
4. If the build fails, do **not** retry with `--force` or `clean`; report the
   root cause and ask whether to investigate or to wipe `build/` and reconfigure.

Refer to the `gz-build` skill for the full set of options.
