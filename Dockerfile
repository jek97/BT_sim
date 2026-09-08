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
# fails downstream (behaviortree_ros2 and this repo's amiga_ros2_behavior_tree
# both hit this). Safe once upstream fixes this -- the glob matches nothing
# and the symlink step is a no-op. Same fix as scripts/ci/build_underlay.sh.
RUN arch_lib="/opt/ros/${ROS_DISTRO}/lib/$(uname -m)-linux-gnu" && \
    if compgen -G "$arch_lib"/libbehaviortree_cpp*.so* > /dev/null; then \
        ln -sf "$arch_lib"/libbehaviortree_cpp*.so* "/opt/ros/${ROS_DISTRO}/lib/"; \
    fi

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
