# AGENTS.md

This file defines repository-level instructions for Codex in this ROS2/robotics project.

## Scope And Priority

- These instructions apply to the whole repository.
- If a deeper `AGENTS.md` exists in a subdirectory, the deeper file overrides this file for files under that path.
- Direct user instructions override this file.

## Project Context

- Domain: ROS2 robotics software with Clean Architecture boundaries.
- Target ROS distro: Humble.
- Primary guidance root: `.codex/`.
- Skills location: `.codex/skills/`.

## Required References

- ROS2 docs: `https://docs.ros.org/en/humble/index.html`
- Codex AGENTS.md guide: `https://developers.openai.com/codex/guides/agents-md`
- Codex skills docs: `https://developers.openai.com/codex/skills`
- Codex rules docs: `https://developers.openai.com/codex/rules`
- OpenAI skills examples: `https://github.com/openai/skills`

When there is a conflict:
1. Prefer explicit user instructions.
2. Prefer ROS2 Humble docs for ROS behavior/APIs.
3. Prefer Codex docs for agent/skills/rules conventions.

## What Codex Should Load First

1. `.codex/CODEX.md`
2. `.codex/rules/*.md`
3. `.codex/rules/default.rules`
4. Relevant skill from `.codex/skills/<skill-name>/SKILL.md`
5. `.codex/commands/ros2.md` when command help is needed

Load only relevant files; avoid loading all skills/rules if not needed.

## Robotics Engineering Constraints

- Keep Clean Architecture boundaries:
  - Domain: no ROS dependencies.
  - Application: orchestrates use-cases.
  - Infrastructure: ROS2 nodes/messages/adapters.
- Prefer deterministic behavior in control loops and callbacks.
- Use ROS2 QoS intentionally for sensor, control, and state topics.
- For TF2, keep ROS message types out of domain entities.
- For hardware/safety paths, fail safely and log actionable diagnostics.

## Implementation Workflow

1. Understand request and identify affected packages/nodes.
2. Select matching skill from `.codex/skills/`.
3. Apply relevant rules from `.codex/rules/`.
4. Implement minimal changes needed.
5. Run targeted validation (`colcon test`, package tests, lint if available).
6. Summarize changes, assumptions, and any unvalidated areas.

## Skill Usage Rules

- Trigger by direct name (for example, `$ros2-dockerfile`) or semantic match.
- Use kebab-case skill names.
- Read `SKILL.md` first; read additional files only when required.
- Keep `.codex/skills` as the source of truth.

## Docker And Deployment

- Follow `.codex/rules/ros2_dockerfile.md` for Dockerfile changes.
- Prefer `ros:humble-ros-base` unless desktop tools are needed.
- Run `rosdep` before `colcon build` inside images.
- Use multi-stage builds for runtime images when possible.

## Command Safety

- Follow `.codex/rules/default.rules` policy.
- Never use destructive commands unless explicitly requested.
- Treat `git push`, Docker changes, and system-level commands as approval-sensitive operations.

## Quality Bar

- Keep edits focused and reversible.
- Do not introduce placeholder code like "TODO implement later" for requested functionality.
- Update nearby docs/instructions when behavior or workflows change.
- If tests are not run, state that clearly.
