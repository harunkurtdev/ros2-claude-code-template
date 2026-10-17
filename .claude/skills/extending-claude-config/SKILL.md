---
name: extending-claude-config
description: Author a new asset for this .claude/ template — a rule, skill, slash command, or sub-agent — following the project's conventions and wiring it into the CLAUDE.md / README.md indexes. Trigger when the user wants to add or extend a command, skill, agent, or rule (make the template itself extensible).
---

# Extending the `.claude/` config

This template is **self-extensible**: rules, skills, slash commands and
sub-agents are plain Markdown files under `.claude/`. This skill is the
recipe for adding a new one *consistently* — same frontmatter, same tone,
same indexing — so the config stays coherent.

> Scope note: for `settings.json` (permissions, hooks, env), use the
> `update-config` skill instead. This skill is about authoring
> rules/skills/commands/agents.

## The four asset types

| Asset | Lives in | Purpose | Discovered by |
|-------|----------|---------|---------------|
| **rule** | `rules/<name>.md` | reference / constraints Claude should always respect | read on demand; listed in CLAUDE.md |
| **skill** | `skills/<name>/SKILL.md` | a how-to / scaffolding recipe with YAML frontmatter | the skill loader (frontmatter `name`/`description`) |
| **command** | `commands/<name>.md` | a `/slash-command` with frontmatter | the command loader |
| **agent** | `agents/<name>.md` | a sub-agent (`subagent_type`) with frontmatter | the Agent tool |

## Conventions (match the existing files)

- **Tone:** concise, imperative, concrete. Tables over prose. Reference
  real source paths (`~/nav2_ws/src/...`) when the asset documents a
  library.
- **Cross-link:** a skill points at its rule(s); a command points at its
  skill; an agent points at the rules it reviews against. Use the exact
  file names.
- **Pairing:** a *scaffolding* skill usually gets a matching slash command
  and, where useful, a *reviewer* agent (e.g. `ros2-controller-creation`
  ↔ `/new-controller` ↔ `ros2-controllers-reviewer`). Reference packages
  cloned next to the workspace are the source of truth — read them.
- **Always update the indexes** (see "Wiring" below) — an unindexed asset
  is invisible.

## Templates

### rule — `rules/<name>.md`
No frontmatter. Start with an H1 and a one-line purpose + source path.
```markdown
# <Title> Reference

<one-line purpose>. Source: `~/nav2_ws/src/<repo>/` (version …).

## 1. <Concept>
| … | … |
```

### skill — `skills/<name>/SKILL.md`
```markdown
---
name: <kebab-name>
description: <what it does + when to trigger; trigger-oriented, one sentence>
---

# <Title>

<intro + pointers to rules/reference repos>

## First decision: <pick the variant>
## <step-by-step>
## Common pitfalls
```

### command — `commands/<name>.md`
```markdown
---
description: <one-line, imperative>
argument-hint: "<arg1> <arg2> [optional]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

<what it does>

Argument handling (`$ARGUMENTS`): 1. … 2. …
If anything is missing, ask before acting.

Process:
1. Read the `<skill>` skill.
2. …
N. Print a summary (files created, next command to run).
```

### agent — `agents/<name>.md`
```markdown
---
name: <kebab-name>
description: Use proactively before <X>. Reviews a diff against <conventions>. Returns a punch list with file:line anchors, not a rewrite.
tools: ["Bash", "Read", "Grep", "Glob"]
model: sonnet
---

You are the <domain> reviewer for …
## Process  (identify diff → cluster → checklist → report)
## Checklist  (grounded in the rules)
## Output format  (Must fix / Should fix / Nice to have; file:line)
Do not restate the diff. Do not rewrite the patch. Stay short.
```

## Wiring (mandatory — do this every time)

After creating the file, index it so it is discoverable:

1. **`.claude/CLAUDE.md`**
   - rule → add a row to the **Rules** table (and "Where things live").
   - skill → add a row to the **Skills index** (right subsection).
   - command → add a row to the **Slash commands** table.
   - agent → add a row to the **Sub-agents** table.
2. **`.claude/README.md`**
   - add the file to the `rules/` or `agents/` tree, or the command list
     under "How to use".
3. If the asset documents an external repo, note the clone path and
   version, exactly as the VDA 5050 / ros2_control / BehaviorTree entries do.

## Checklist before finishing

- [ ] Frontmatter present & valid (skills/commands/agents).
- [ ] Cross-links use real existing file names.
- [ ] Indexed in **both** CLAUDE.md and README.md.
- [ ] Tone/sections match a sibling asset of the same type.
- [ ] For scaffolding skills: a matching command exists (and a reviewer
      agent if it warrants review).
