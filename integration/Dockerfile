FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-numpy python3-pytest python3-scipy \
    ros-jazzy-sensor-msgs-py ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v1.15.0 https://github.com/PX4/px4_msgs.git \
      /opt/px4_ws/src/px4_msgs \
    && . /opt/ros/jazzy/setup.sh \
    && cd /opt/px4_ws \
    && colcon build --merge-install

COPY integration/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /autonomy
