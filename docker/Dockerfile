# syntax=docker/dockerfile:1
# Unified control + dynamics environment. Building does not launch ROS nodes or UR5.
FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Official Noble/amd64 repository versions verified on 2026-10-26.
# No fallback to a different release if the selected packages disappear.
ARG BRIDGE_VERSION=0.1.2-1noble.20261006.035710
ARG DEMOS_VERSION=0.1.2-1noble.20261006.040614
ARG MSGS_VERSION=0.1.2-1noble.20260911.141240
ARG PLUGINS_VERSION=0.1.2-1noble.20260924.184447
ARG VENDOR_VERSION=0.1.0-1noble.20260826.114141
RUN apt-get update && apt-get install --no-install-recommends -y \
        "ros-jazzy-mujoco-ros2-control=${BRIDGE_VERSION}" \
        "ros-jazzy-mujoco-ros2-control-demos=${DEMOS_VERSION}" \
        "ros-jazzy-mujoco-ros2-control-msgs=${MSGS_VERSION}" \
        "ros-jazzy-mujoco-ros2-control-plugins=${PLUGINS_VERSION}" \
        "ros-jazzy-mujoco-vendor=${VENDOR_VERSION}" \
        ros-jazzy-controller-manager \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-position-controllers \
        ros-jazzy-control-toolbox \
        python3-venv libgl1 libglfw3 libosmesa6 \
        libgl1-mesa-dri libegl1 libglx-mesa0 \
    && dpkg-query -W > /opt/piakf-system-packages.txt \
    && rm -rf /var/lib/apt/lists/*

# ROS Python uses Ubuntu Python 3.12. The second venv isolates Pinocchio's NumPy.
ENV VIRTUAL_ENV=/opt/piakf-venv \
    PATH=/opt/piakf-venv/bin:${PATH} \
    REGRESSION_PYTHON=/opt/piakf-rigid-venv/bin/python \
    PYTHONNOUSERSITE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    MUJOCO_GL=osmesa \
    MPLBACKEND=Agg \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_DOMAIN_ID=97

COPY requirements/ /opt/piakf-requirements/
RUN --mount=type=cache,target=/var/cache/piakf-pip \
    /usr/bin/python3 -m venv --system-site-packages "${VIRTUAL_ENV}" \
    && python -m pip install --cache-dir /var/cache/piakf-pip --only-binary=:all: \
        -r /opt/piakf-requirements/requirements-main.txt \
    && python -m pip install --cache-dir /var/cache/piakf-pip --only-binary=:all: \
        --index-url https://download.pytorch.org/whl/cpu \
        -r /opt/piakf-requirements/requirements-torch.txt \
    && python -m pip check \
    && python -m pip freeze > /opt/piakf-python-packages.txt
RUN --mount=type=cache,target=/var/cache/piakf-pip \
    /usr/bin/python3 -m venv /opt/piakf-rigid-venv \
    && "${REGRESSION_PYTHON}" -m pip install --cache-dir /var/cache/piakf-pip --only-binary=:all: \
        -r /opt/piakf-requirements/requirements-rigid.txt \
    && "${REGRESSION_PYTHON}" -m pip check \
    && "${REGRESSION_PYTHON}" -m pip freeze > /opt/piakf-rigid-python-packages.txt

COPY checks/check-environment.py /opt/piakf-check-environment.py
COPY checks/check-rigid-environment.py /opt/piakf-check-rigid-environment.py
COPY --chmod=0755 entrypoint.sh /usr/local/bin/piakf-entrypoint
# Import/version/link checks and the existing small rigid-library installation check.
# No mj_step, ROS node, control demo, project trajectory or learned model is executed.
RUN /usr/local/bin/piakf-entrypoint python /opt/piakf-check-environment.py \
        --output /opt/piakf-installation-check.json \
    && "${REGRESSION_PYTHON}" /opt/piakf-check-rigid-environment.py

WORKDIR /workspace
USER 1000:1000
ENTRYPOINT ["/usr/local/bin/piakf-entrypoint"]
CMD ["bash"]
