FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

# ROS 2 Humble is already configured by the base image. Add the OSRF repository
# because Humble + Gazebo Harmonic integration packages are published there.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release sudo \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git bash-completion \
      python3-colcon-common-extensions python3-rosdep python3-pip python3-venv python3-pytest python3-yaml \
      python3-numpy python3-opencv \
      geographiclib-tools \
      ros-humble-mavros ros-humble-mavros-extras \
      ros-humble-octomap-server \
      ros-humble-ros-gzharmonic \
      ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 \
      ros-humble-cv-bridge ros-humble-message-filters \
      ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
      ros-humble-image-transport \
    && source /opt/ros/humble/setup.bash \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir --upgrade pip wheel \
    && /opt/venv/bin/python -m pip install --no-cache-dir "numpy>=1.23,<2" \
    && /opt/venv/bin/python -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && /opt/venv/bin/python -m pip install --no-cache-dir "ultralytics>=8.2,<9" \
    && rm -rf /var/lib/apt/lists/* /root/.cache/pip

# ROS Humble cv_bridge is built against the NumPy 1.x ABI.
# Ultralytics may otherwise pull OpenCV 4.12+ and NumPy 2.x.
RUN /opt/venv/bin/python -m pip install --no-cache-dir --force-reinstall \
      "numpy==1.26.4" \
      "opencv-python==4.11.0.86"

# Match the host UID/GID so bind-mounted workspace files remain owned by the user.
RUN if getent group "${USER_GID}" >/dev/null; then \
      EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1)"; \
    else \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
      EXISTING_GROUP="${USERNAME}"; \
    fi \
    && useradd --uid "${USER_UID}" --gid "${EXISTING_GROUP}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY docker/entrypoint.sh /usr/local/bin/drone-entrypoint
RUN chmod +x /usr/local/bin/drone-entrypoint

ENV PATH=/opt/venv/bin:$PATH \
    VIRTUAL_ENV=/opt/venv \
    PYTHONPATH=/opt/venv/lib/python3.10/site-packages \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=42 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    QT_X11_NO_MITSHM=1 \
    PYTHONUNBUFFERED=1

USER ${USERNAME}
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/drone-entrypoint"]
CMD ["bash"]
