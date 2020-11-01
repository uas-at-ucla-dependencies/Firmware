# UAS@UCLA PX4 Firmware

# Archived Version of this README

# Intro to MAVLink, QGroundControl, and the PX4 Simulator

# Contents
* [Overview](/README.md#overview)
* [Installing QGroundControl](/README.md#installing-qgroundcontrol)
* [Setting Up the Simulator](/README.md#setting-up-the-simulator)
* [Using QGroundControl](/README.md#using-qgroundcontrol)
  * [Establishing Communication](/README.md#establishing-communication)
  * [Units of Measurement](/README.md#units-of-measurement)

# Overview
* [MAVLink](https://mavlink.io/en/): "a very lightweight messaging protocol for communicating with drones"
* [QGroundControl](http://qgroundcontrol.com/): "provides full flight control and mission planning for any MAVLink enabled drone"
* [PX4](https://px4.io/): "an open source flight control software for drones and other unmanned vehicles"
  * [PX4 Simulation](https://dev.px4.io/v1.9.0/en/simulation/index.html): Use the PX4 Firmware in a simulated environment to enable [HITL and/or SITL](https://www.quora.com/What-is-Software-in-the-Loop-SIL) testing.
    * [Gazebo](https://dev.px4.io/v1.9.0/en/simulation/gazebo.html): One of several simulators compatible with PX4 (which we use)

## Diagram
![mavlink-qground-px4-diagram.png](https://uasatucla.org/images/subsystems/controls/mavlink-qgound-px4sim/mavlink-qground-px4-diagram.png)

# Installing QGroundControl
Installing is easy!
**[Installation Instructions](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)**

You can also get the iOS/Android app for fun.

# Setting Up the Simulator

## Running with Docker
You can install Docker from https://docs.docker.com/get-docker.

Download the pre-made Docker image (no need to clone the GitHub repo)
```bash
docker pull uasatucla/px4-simulator
```

Run a Docker container and start the simulator:
```bash
docker run -it --rm --name px4-simulator -p 18570:18570/udp -p 5760:5760/tcp uasatucla/px4-simulator ./run.sh simulate_headless
```
*Temporary note: I'm interested in making an "integration-testing" repository that would enable testing integration between px4, controls code, ground code, vision code, etc. Once that exists people won't have to copy that giant command into their terminal.*

The general usage of the script is:
```bash
./run.sh [action] [frame_type] [location] # 'action' is reqiured
```
For more details, [take a look at the script](https://github.com/uas-at-ucla-dependencies/PX4-Autopilot/blob/release/1.11-uasatucla/run.sh).

### Explanation
The ```simulate_headless``` action runs the PX4 Firmware, which is fed simulated data by Gazebo. You can also use the ```simulate``` action (without "headless") to get a nice GUI that shows the simulated drone, at the expense of more computing resources. Showing a display through Docker has not been set up yet (see below for running natively), but you often don't need it.

## Running Natively
Running with Docker is the easiest, but you can also try running directly on your machine.
[Clone the UAS@UCLA PX4 Firmware repository](https://github.com/uas-at-ucla-dependencies/PX4-Autopilot.git), which contains the PX4 Firmware and scripts for setting up simulation.
### MacOS
**Setup:**
[PX4 Setup Instructions](https://dev.px4.io/v1.11/en/setup/dev_env_mac.html)
[SITL Gazebo Setup Instructions](https://github.com/PX4/PX4-SITL_gazebo#installation-gazebo-9)
* Only follow installation instructions, you do not have to build anything yet.
If you have trouble with Python, you can try setting up a virtual environment, which the following script does automatically:
```bash
./setup_python_venv.sh
```
Finally, compile the code:
```bash
./build.sh
```

**Run:**
```bash
./run.sh simulate_headless
```

**Clean your build:**
Sometimes a good thing to do if you run into problems building:
```make clean```

### Linux & Raspberry Pi
The following may work on desktop Ubuntu, although it hasn't been tested. (There is also no support for Ubuntu 19 as of writing this, and you might as well use Docker if on desktop Linux.)
```bash
./setup_debian.sh # install px4 & gazebo dependiencies
./setup_python_venv.sh # setup python virtual env & install python dependencies
./build.sh
./run.sh simulate_headless
```
This *has* been tested on [Raspbian Stretch](http://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2019-04-09/) on Raspberry Pi (no support for Buster yet). It takes a while (few hours) so you'll want to run everything at once and leave it running:
`./setup_debian.sh && ./setup_python_venv.sh && ./build.sh && ./run.sh simulate_headless`
Optionally, you could use [screen](https://raspi.tv/2012/using-screen-with-raspberry-pi-to-avoid-leaving-ssh-sessions-open) to allow you to close the terminal while it runs.

**Warning:** It's possible to encounter network timeouts in setup_python_venv.sh. If this happens you will just have to re-run it until it works.

You can also just flash [this disk image](https://drive.google.com/open?id=1LZpyXgj1KRLo2XchZBEVLvgntEhZt0Yn) with everything already set up to the SD card. *Note that the disk image is a fixed size, regardless of your SD card capacity, unless you look up how to expand it. This shouldn't matter if you don't plan on using the image for anything else, but you should prefer the "16GB" one (only a 3.4GB download) if your SD card is at least that size.*

**There are a few reasons why you might use Raspberry Pi for the simulator:**
1. You have a Windows computer and don't want to run a virtual machine.
2. A Pi can be easily shared around so people don't have to spend time installing/building on their own machines.
3. It more accurately models having the flight controller software and ground control software on different machines.

# Using QGroundControl
## Establishing Communication
If you run QGroundControl on the same computer as the simulator, QGroundControl may automatically connect to it. However, it is important to know how to connect to a remote machine, whether it be a simulator or the real drone. You can also use these instructions if QGroundControl does not automatically connect.

The PX4 Firmware uses TCP to communicate with the Gazebo simulator and UDP to communicate with outside world, using [the ports documented here](https://dev.px4.io/master/en/simulation/#default-px4-mavlink-udp-ports) (scroll down on that page to see a nice diagram).

As per the documentation, we will be listening on UDP port 14550 in QGroundControl. We also need to know that the port used to send and receive on the other end (where the PX4 Firmware is running) is 18570.

### Instructions
1. Click on the `Q` icon in the top left in QGroundControl.
2. Select `Comm Links` and then `Add` at the bottom of the screen.
3. In the Type dropdown, select `UDP`.
4. The Listening Port should default to 14550.
5. Click `Add` under Target Hosts.
6. Type in `<address>:18570`, where `<address>` is the IP address or hostname of the machine the simulator is running on (`localhost` if it's the same computer). For example, if you are using a Raspberry Pi plugged in to your computer or on your local network, enter `raspberrypi.local:18570`. Note that if you use a hostname like this, it will be converted to the IPv4 address before the configuration is saved.
7. Click `Ok`.
8. Start the simulator and click `Connect` from the `Comm Links` tab.
9. You should see the state of the drone in the status bar, and no longer see "Waiting for Vehicle Connection."
![qground-connection.png](https://uasatucla.org/images/subsystems/controls/mavlink-qgound-px4sim/qground-connection.png)

### How This Works
#### MAVLink Messages
All the messages sent between QGroundControl and the PX4 Firmware are [MAVLink messages](https://mavlink.io/en/messages/common.html#messages).
#### How a Connection is Established
The connection is established like this:
1) QGroundControl sends initial message to simulator w/ destination port **18570**
2) Simulator is listening on this port, so it receives the message and thinks, "Gotcha, I will send all my data to you w/ destination port **14550**," and proceeds to send data.
3) QGroundControl is able to receive data and thinks, "Great! Glad I was listening on port **14550**."

Because of how this works, even if you close QGroundControl, the simulator will still be sending data to your computer. If you were to reopen QGroundControl, it would automatically reconnect since it automatically listens on UDP port 14550.

Similarly, QGroundControl remembers its last connection, so even if you restart the simulator, QGroundControl would be looking for it and automatically re-establish the connection. These features are a result of the AutoConnect feature in QGrounControl, which can be turned off from Application Settings.

Only if you restart the simulator AND QGroundControl will you have to manually reconnect.

#### Unique Connection
The simulator can only establish a link with one "partner" or ground station. When you establish a connection, the simulator will log your IP address:
```
INFO  [mavlink] partner IP: 192.168.0.11
```
After this, the simulator is forever locked into sending data to that IP address. You cannot connect from a different computer unless you restart the simulator.

### MAVLink Router
Obviously, it could be helpful to connect multiple computers to the drone, so in practice we use [MAVLink Router](https://github.com/intel/mavlink-router), which connects to the drone and can route the MAVLink data stream to multiple IP addresses. Specifically, you can send data via UDP to pre-configured IP addresses, and it also sets up a TCP server which accepts connections from anyone.

The best way to run MAVLink Router is with the following command:
```bash
./run.sh mavlink_router
```
If you're using Docker and started the simulator as in [Running with Docker](#running-with-docker):
```bash
docker exec -it px4-simulator ./run.sh mavlink_router
```
This runs `mavlink-routerd 0.0.0.0:14550`.

A more advanced use of MAVLink Router would be something like `mavlink-routerd -e 192.168.3.20:9011 0.0.0.0:14550`, where one or more destination addresses are set with `-e`, and the last argument is the address of the flight controller.

MAVLink Router automatically listens on TCP port **5760**. To connect to this (from any computer), create a new Comm Link as in the [previous instructions](/README.md#instructions), except this one will be of type `TCP`. The TCP Port should default to 5760, and the Host Address needs to be set to the IP address of the computer running MAVLink Router.

Note that MAVLink Router does not work the same way for UDP. You need to specifiy the IP address of each computer you want to receive data over UDP using `-e` as noted above. One reason you might care about the difference between UDP and TCP is that UDP is lighter-weight protocol more suited to streaming data.

#### The Weirdness of Broadcasting over UDP
The following is not a typical way to use MAVLink Router, but may be interesting for advanced users. There are a couple ways to specify a *broadcast address* in MAVLink Router. Both of these commands have the same effect (or so it seems, as it isn't documented anywhere):
```bash
mavlink-routerd -e 255.255.255.255:9010 0.0.0.0:14550
```
```bash
mavlink-routerd -e :9010 0.0.0.0:14550
```
This will broadcast the MAVLink data via UDP to all machines on the local network (This is the effect of sending to the IP address 255.255.255.255). **However, as soon as MAVLink Router receives a connection from QGroundControl, it will stop broadcasting and start communicating solely with the connected computer.**

To connect from QGroundControl, create a new UDP Comm Link with the relevant Listening Port (**9010** in this case). You do not need to add any Target Hosts.

In effect, this can be used when you do not initially know the IP address of the computer on which you are running QGroundControl. To use multiple computers this way, simply specifiy multiple ports using multiple `-e` statements.

## Units of Measurement
From the AUVSI SUAS rules, the ground station display "must indicate the UAS speed in KIAS or ground speed in knots" (KIAS is just knots but specifically refers to airspeed). You can switch to these units from General Settings (click on the `Q` icon).
