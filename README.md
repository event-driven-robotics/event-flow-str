# event-flow-str: Event-based Optical Flow Using Spatio-temporal Registration
Code associated with "Event-based Optical Flow Using Spatio-temporal Registration", Z. Li et. al. TPAMI 2026

[Project website](https://event-driven-robotics.github.io/event-flow-str/).

Dataset: https://zenodo.org/records/22894730

```
@ARTICLE{11673316,
  author={Li, Zhichao and Glover, Arren and Natale, Lorenzo and Bartolozzi, Chiara},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence}, 
  title={Event-based Optical Flow Using Spatio-temporal Registration}, 
  year={2026},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TPAMI.2026.3729469}}

```

## camera setup
This setup is for Prophesee Gen3

### run the container for the camera

Run once

```bash
docker run --name atis3-driver -it \
  --network host \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rwm' \
  eventdrivenrobotics/atis-gen3:focal_yarp_v3.8.1_ed_vmain

```

### Run yarpserver

```bash
yarpserver
```

### Configure YARP inside the container

```bash
yarp conf 172.17.0.1 10000
```

Inside the container, configure YARP to use the name server. Replace `172.17.0.1` and `10000` with the actual name-server address if different:


### Receive events data
```bash
atis-bridge-sdk --s 40
```

`s` control the sensitivity



## Optical flow Environment Setup
This will guide you how to build and run 'event-flow-str' in Docker, connect it to YARP event-camera stream, and display the OpenCV visualization on a linux host.


### Prerequisites

- Linux host with Docker installed and usable by the current user.
- An X11 desktop session (only required for the OpenCV window).
- A running YARP name server reachable by both the event-camera driver and the container.
- An event-camera driver publishing an `ev::AE` output port. In the examples below, this port is `/atis3/AE:o`.


### Build the image

```bash
docker build -t event-flow-test .
```

### Start the YARP name server

If no YARP name server is started. 

```bash
yarpserver
```

Check the server and the camera source port.

```bash
yarp name list
```

Expected extires include `/root` (the name server) and the camera output, for example `/atis3/AE:o`.


### Allow GUI access

Allow the container's root user to access the local X server:

```bash
xhost +local:root
```

### Run the container

Run once

```bash
docker run --name flow-app -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD:/workspace" \
  event-flow-test bash
```

### Configure YARP inside the container

Inside the container, configure YARP to use the name server. Replace `172.17.0.1` and `10000` with the actual name-server address if different:

```bash
yarp conf 172.17.0.1 10000
yarp check
yarp name list
```

### Build and start the application

Inside the container:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/event-flow-str --name /event-flow-str
```

The application creates the YARP input port:

```text
/event-flow-str/AE:i
```

### Connect the event camera

Connect the camera source to the application input:

```bash
yarp connect /atis3/AE:o /event-flow-str/AE:i
```

Verify that the connection exists:

```bash
yarp exists /atis3/AE:o /event-flow-str/AE:i
echo $?
```

An exit code of `0` means the connection is active. The application window should then receive events and show the optical-flow visualisation.


### Run evaluation on DSEC

The following example tests the estimation accuracy:

```bash
./build/dsec-app \
    ./sampledata/events.h5 \
    ./sampledata/rectify_map.h5 \
    ./sampledata/interlaken_00_b.csv
```

To load .h5 files, the Blosc filter plugin is required. Follow these [intructions](https://github.com/tub-rip/events_h52bag) to install it. 


