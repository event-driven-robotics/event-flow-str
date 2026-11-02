# Build and test image for event-flow-str.
# Keep the original Ubuntu/YCM/YARP environment, but take event-driven source
# from the same pinned base used by the ATIS3 camera-driver image.
ARG EVENT_DRIVEN_IMAGE=eventdrivenrobotics/event-driven:focal_yarp_v3.8.1_ed_vmain
FROM ${EVENT_DRIVEN_IMAGE} AS event_driven_source

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ARG DEBIAN_FRONTEND=noninteractive
ARG CODE_DIR=/usr/local/src

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    build-essential \
    cmake \
    git \
    pkg-config \
    libace-dev \
    libeigen3-dev \
    libopencv-dev \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# Keep the original YCM version.
ARG YCM_VERSION=v0.15.2
RUN git clone --depth 1 --branch "${YCM_VERSION}" \
      https://github.com/robotology/ycm.git "${CODE_DIR}/ycm" \
    && cmake -S "${CODE_DIR}/ycm" -B "${CODE_DIR}/ycm/build" \
    && cmake --build "${CODE_DIR}/ycm/build" --parallel "$(nproc)" \
    && cmake --install "${CODE_DIR}/ycm/build"

# Keep the original YARP version. Serial building avoids the ROS-message
# generation race encountered with YARP 3.8.
ARG YARP_VERSION=v3.8.0
RUN git clone --depth 1 --branch "${YARP_VERSION}" \
      https://github.com/robotology/yarp.git "${CODE_DIR}/yarp" \
    && cmake -S "${CODE_DIR}/yarp" -B "${CODE_DIR}/yarp/build" \
    && cmake --build "${CODE_DIR}/yarp/build" --parallel 1 \
    && cmake --install "${CODE_DIR}/yarp/build" \
    && ldconfig

# Do not clone a floating `main` branch. Copy the exact event-driven source
# revision used by the ATIS3 driver base image and build it in a clean tree.
COPY --from=event_driven_source /usr/local/src/event-driven \
     ${CODE_DIR}/event-driven-source
RUN cmake -S "${CODE_DIR}/event-driven-source" \
          -B "${CODE_DIR}/event-driven-build" \
    && cmake --build "${CODE_DIR}/event-driven-build" --parallel "$(nproc)" \
    && cmake --install "${CODE_DIR}/event-driven-build" \
    && ldconfig

WORKDIR /workspace

# Mount the checked-out pull-request source at /workspace.
CMD ["bash"]
