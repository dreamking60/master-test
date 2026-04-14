FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        iputils-ping \
        iproute2 \
        net-tools \
        tcpdump \
        python3-scapy \
        ros-jazzy-sros2 \
        ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/project

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
