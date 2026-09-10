ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full-2025-12-07

FROM ${BASE_IMAGE} AS base

ARG WORKSPACE_ROOT="/amiga-ros2-bridge"
ARG PACKAGE_NAME="amiga_ros2_bridge"
ARG MACHINE_NAME="agx"

# any utilities you want
RUN apt-get update && apt-get install -y git wget curl python3-full python3-pip vim net-tools netcat-traditional build-essential cmake \
    ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-behaviortree-cpp ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-trajectory-controller ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-gripper-controllers ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-diagnostic-updater \
    tmux \
    rhash librhash-dev \
    byacc flex

# Upstream regression in the humble build of ros-humble-behaviortree-cpp: it
# installs libbehaviortree_cpp.so* into the multiarch lib dir
# (lib/<arch>-linux-gnu), but its own ament export's find_library() call has
# no LIBRARY_DIRS and only searches the flat lib/, so find_package() for it
# fails downstream (behaviortree_ros2 and every package depending on it, this
# repo's own amiga_ros2_behavior_tree and ros2-kortex-control's kortex_bt
# included, both hit this). Safe once upstream fixes this -- the glob then
# matches nothing and the loop body never runs.
#
# A `for`/`[ -e ]` loop, NOT `compgen -G` (scripts/ci/build_underlay.sh's own
# approach, which only works there because that script has a `#!/bin/bash`
# shebang and is invoked directly): Docker's RUN always executes via
# `/bin/sh -c`, which on this (Debian-based) image is dash, not bash --
# `compgen` is a bash builtin with no dash equivalent, so under `sh -c` the
# `if compgen ...; then` line silently evaluates to "command not found" (a
# non-zero exit), the `if` treats that as false, and the whole symlink step
# was a no-op on EVERY build with no error raised -- exactly the failure
# this fix is for, just never actually applied. A `for f in pattern; do
# [ -e "$f" ] && ...; done` loop is POSIX/dash-safe: an unmatched glob stays
# literal, `[ -e ]` on a literal non-existent path is simply false, so it
# degrades to a correct no-op instead of a silent bash-only no-op.
RUN arch_lib="/opt/ros/${ROS_DISTRO}/lib/$(uname -m)-linux-gnu" && \
    for so_file in "$arch_lib"/libbehaviortree_cpp*.so*; do \
        [ -e "$so_file" ] && ln -sf "$so_file" "/opt/ros/${ROS_DISTRO}/lib/"; \
    done; true

# The SPIN model checker, used by amiga_ros2_agents/verification/verify.py to re-verify a
# replanned mission. Same script CI runs, so both environments get the same
# checker; see the header there for why it is a source build.
COPY scripts/ci/install_spin.sh /tmp/install_spin.sh
RUN SKIP_APT=1 /tmp/install_spin.sh && rm /tmp/install_spin.sh

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN . /.venv/bin/activate && \
    pip install -r /requirements.txt

WORKDIR ${WORKSPACE_ROOT}

COPY manifests/ /manifests/
RUN rosdep update && rosdep install --from-paths /manifests --ignore-src -r -y

RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

COPY cyclonedds.xml /amiga-ros2-bridge/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///amiga-ros2-bridge/cyclonedds.xml
