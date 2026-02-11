---
name: ros2-dockerfile
description: Use when creating or updating ROS2 Humble Dockerfiles for development, CI, or runtime images.
---

# ROS2 Dockerfile Skill
## Authoritative Sources

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- Codex Skills docs: `https://developers.openai.com/codex/skills`
- ROS2 tutorials: `https://docs.ros.org/en/humble/Tutorials.html`
- Codex skill conventions and examples: `https://github.com/openai/skills`

Use ROS2 Humble docs as source of truth for dependency and build behavior.

## When To Use

- Build a new Dockerfile for ROS2 packages/workspaces.
- Convert local build flow (`rosdep`, `colcon`) into container flow.
- Create dev/runtime split images or CI-friendly images.

## Workflow

1. Choose base image (`ros:humble-ros-base` by default).
2. Install system/build dependencies in one apt layer and clean apt cache.
3. Copy workspace source and run `rosdep install` from `/ws/src`.
4. Build with `colcon build --merge-install` (add `--symlink-install` for dev).
5. Add entrypoint that sources `/opt/ros/humble/setup.bash` and workspace install setup.
6. Switch to non-root user for runtime execution.

## Dev Dockerfile Template

```dockerfile
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install --symlink-install

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

## Runtime Multi-Stage Template

```dockerfile
FROM ros:humble-ros-base AS builder
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

COPY src /ws/src
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install

FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY --from=builder /ws/install /ws/install
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "my_bringup", "system.launch.py"]
```

## Entrypoint Template

```bash
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
```

## Checklist

- `rosdep install` runs successfully in image build.
- `colcon build` completes in container.
- Container starts with ROS environments sourced.
- Image runs without root for application process where possible.
