---
description: Author a new skill under .claude/skills/<name>/SKILL.md with valid frontmatter, following the template conventions, and index it in CLAUDE.md + README.md.
argument-hint: "<skill_name> [one-line description]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Add a new skill to this `.claude/` config.

Argument handling (`$ARGUMENTS`):
1. `<skill_name>` — `kebab-case`: lowercase letters, digits, and hyphens
   (required); becomes the directory name and the frontmatter `name`.
2. `[description]` — one-line, trigger-oriented summary. If missing, ask.

If anything is unclear (does it pair with a command/agent? which rules?),
ask before writing.

Process:
1. Read the `extending-claude-config` skill (conventions + templates).
2. Verify `skills/<skill_name>/` does not already exist.
3. Create `skills/<skill_name>/SKILL.md` with:
   * YAML frontmatter `name:` + `description:` (the description must say
     **what it does and when to trigger**).
   * a body: intro + pointers to the relevant `rules/` and any reference
     repo under `~/nav2_ws/src/`, a "first decision" section if it has
     variants, step-by-step instructions, and a "Common pitfalls" list.
   * tone/sections matching a sibling skill (e.g.
     `ros2-controller-creation`).
4. **Index it**:
   * add a row to the **Skills index** in `.claude/CLAUDE.md` (correct
     subsection).
   * if it is a reference-type skill, also consider a "Where things live"
     row.
5. If it is a *scaffolding* skill, offer to also create the matching
   `/new-*` command (`/new-command`) and, if it warrants review, a
   reviewer agent (`/new-agent`).
6. Print: file created + the index rows added.

Frontmatter is mandatory — a skill without `name`/`description` is not
discoverable. Always update the index in the same change.
