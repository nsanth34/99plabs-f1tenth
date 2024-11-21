ARG ROS_DISTRO=foxy
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base AS builder
ARG TARGETOS TARGETARCH
SHELL ["/bin/bash", "-c"]
WORKDIR /project_ws

COPY src/ src/

RUN mkdir -p install; \
    apt update -qq --no-install-recommends; DEBIAN_FRONTEND='noninteractive' apt install -y --no-install-recommends \
    python3-venv ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-serial-driver -y; \
    python3 -m venv install/venv; source install/venv/bin/activate; touch install/venv/COLCON_IGNORE; \
    test -f src/requirements.txt && pip install -r src/requirements.txt; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build; source install/setup.bash; \
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y; \
    export PYVER=$(python3 -V | awk '{print $2}' | cut -d'.' -f1-2); \
    echo "export PYTHONPATH=/project_ws/install/venv/lib/python${PYVER}/site-packages:/opt/ros/${ROS_DISTRO}/lib/python${PYVER}/site-packages" >> install/venv/bin/activate

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-core
SHELL ["/bin/bash", "-c"]

RUN apt update -qq --no-install-recommends; DEBIAN_FRONTEND='noninteractive' apt install -y --no-install-recommends \
    libsdl2-2.0-0 libsdl2-dev; \
    apt autoclean; apt autoremove

COPY --from=builder /project_ws/install/ /project_ws/install/
COPY --from=builder /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}
COPY scripts/envfile /project_ws/install/
COPY scripts/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh
