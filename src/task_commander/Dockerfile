ARG ROS_DISTRO=humble
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as builder
ARG TARGETOS TARGETARCH
SHELL ["/bin/bash", "-c"]

WORKDIR /project_ws

COPY src/ src/

RUN apt update -qq --no-install-recommends; DEBIAN_FRONTEND='noninteractive' apt install -y --no-install-recommends \
    python3-pip python3-venv libeigen3-dev \
    ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-serial-driver; \
    python3 -m venv install/venv; source install/venv/bin/activate; touch install/venv/COLCON_IGNORE; \
    test -f src/requirements.txt && pip install -r src/requirements.txt; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build; source /project_ws/install/setup.bash; \
    rosdep install -i --from-path /project_ws/src --rosdistro ${ROS_DISTRO} -y; \
    export PYVER=$(python3 -V | awk '{print $2}' | cut -d'.' -f1-2); \
    echo "export PYTHONPATH=/project_ws/install/venv/lib/python${PYVER}/site-packages:/opt/ros/humble/lib/python${PYVER}/site-packages" >> install/venv/bin/activate

          
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-core
SHELL ["/bin/bash", "-c"]

COPY --from=builder /project_ws/install/ /project_ws/install/
COPY --from=builder /opt/ros/humble /opt/ros/humble
COPY scripts/envfile /project_ws/install/
COPY scripts/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh
