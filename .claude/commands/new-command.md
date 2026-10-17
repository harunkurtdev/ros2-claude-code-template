---
description: Author a new slash command under .claude/commands/<name>.md with valid frontmatter, following the template conventions, and index it in CLAUDE.md + README.md.
argument-hint: "<command_name> [paired_skill] [one-line description]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Add a new `/slash-command` to this `.claude/` config.

Argument handling (`$ARGUMENTS`):
1. `<command_name>` — `kebab-case` (required); becomes the file name and
   the `/command` name.
2. `[paired_skill]` — the skill this command drives (most commands read a
   skill, then act). If missing, ask.
3. `[description]` — one-line imperative summary.

If anything is unclear, ask before writing.

Process:
1. Read the `extending-claude-config` skill (conventions + templates).
2. Verify `commands/<command_name>.md` does not already exist.
3. Create `commands/<command_name>.md` with:
   * frontmatter: `description:` (one-line, imperative),
     `argument-hint:` (`"<arg1> <arg2> [optional]"`),
     `allowed-tools:` (usually `["Bash","Read","Write","Edit"]`).
   * body: a short intro, an **Argument handling (`$ARGUMENTS`)** section,
     a "if anything is missing, ask first" line, and a numbered
     **Process** that (a) reads the paired skill, (b) does the work, and
     (c) prints a summary (files created, the next command to run).
   * match the structure of a sibling command (e.g. `new-node.md`,
     `new-controller.md`).
4. **Index it**: add a row to the **Slash commands** table in
   `.claude/CLAUDE.md`, and to the command list in `.claude/README.md`
   under "How to use".
5. Print: file created + the index rows added.

A command should defer the real "how" to a skill and stay an
orchestration recipe. Always update both indexes in the same change.
