---
description: Author a new sub-agent under .claude/agents/<name>.md with valid frontmatter (name/description/tools/model), following the template conventions, and index it in CLAUDE.md + README.md.
argument-hint: "<agent_name> [reviewer|architect] [domain]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Add a new sub-agent to this `.claude/` config.

Argument handling (`$ARGUMENTS`):
1. `<agent_name>` — `kebab-case` (required); becomes the file name and the
   `subagent_type`.
2. `[role]` — `reviewer` (pre-PR diff auditor) or `architect` (design
   advisor). Default `reviewer`.
3. `[domain]` — what it covers (e.g. "ros2_control controllers"). If
   missing, ask.

If anything is unclear (which rules does it review against?), ask first.

Process:
1. Read the `extending-claude-config` skill (conventions + templates) and
   skim a sibling agent of the same role
   (`ros2-style-reviewer`/`vda5050-reviewer` for reviewers,
   `clean-arch-architect`/`ecs-architect` for architects).
2. Verify `agents/<agent_name>.md` does not already exist.
3. Create `agents/<agent_name>.md` with:
   * frontmatter: `name:`, `description:` (start with "Use proactively
     before …"; end reviewers with "Returns a punch list with file:line
     anchors, not a rewrite."), `tools:` (`["Bash","Read","Grep","Glob"]`
     for read-only reviewers/architects), `model: sonnet`.
   * body for a **reviewer**: a one-paragraph role statement that names
     the `rules/` it grounds on, a **Process** (identify diff → cluster →
     checklist → report), a **Checklist** organized by concern, and an
     **Output format** (`Must fix` / `Should fix` / `Nice to have`, each
     `file:line — observation — suggestion`). End with "Do not restate the
     diff. Do not rewrite the patch. Stay short."
   * body for an **architect**: trade-off framing, decision questions, and
     "returns a recommendation with trade-offs, not implementation".
4. **Index it**: add a row to the **Sub-agents** table in
   `.claude/CLAUDE.md` and to the `agents/` tree in `.claude/README.md`.
5. Print: file created + the index rows added.

Ground every reviewer checklist item in an actual rule/skill so the
feedback is concrete. Always update both indexes in the same change.
