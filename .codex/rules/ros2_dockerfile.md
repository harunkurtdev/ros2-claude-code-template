---
description: ROS2 Dockerfile Standards (Humble)
---

# ROS2 Dockerfile Standards
## Authoritative Sources

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- ROS2 package development guide: `https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html`
- OpenAI skill conventions: `https://github.com/openai/skills`

When this file conflicts with external references, prefer ROS2 Humble docs for ROS behavior and dependency setup.

## Base Image Policy

- Prefer official ROS images, starting with `ros:humble-ros-base`.
- Use `ros:humble-desktop` only when GUI tooling is required.
- Keep architecture explicit when needed (`--platform=linux/amd64` or `linux/arm64`).

## Dockerfile Layering Rules

- Put apt dependency install before source copy to maximize cache reuse.
- Use one `apt-get update` + `apt-get install` layer, then clean apt lists in the same layer.
- Do not run `apt upgrade` inside Dockerfiles unless explicitly required.
- Copy dependency manifests (`package.xml`, `requirements.txt`) before full source.

## Dependency Rules (ROS2 + Colcon)

- Always run `rosdep install` against `src` before `colcon build`.
- Use `--merge-install` and `--symlink-install` for dev images.
- Use `--merge-install` without symlinks for runtime-focused images.
- Source `/opt/ros/humble/setup.bash` before `rosdep` and `colcon` commands.

## Runtime and Security Rules

- Create and run as a non-root user for normal runtime operations.
- Keep entrypoint deterministic: source ROS environment, then `exec "$@"`.
- Do not hardcode secrets or tokens in Dockerfiles or image layers.

## Minimal Dev Dockerfile Pattern

```dockerfile
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install --symlink-install
```

## Multi-Stage Build Guideline

- Stage 1 (`builder`): install build toolchain + build workspace.
- Stage 2 (`runtime`): copy only install artifacts and runtime deps.
- Runtime stage should avoid compilers unless needed by runtime plugins.
