# ROS in a docker container on a Pi 5

The scripts in this directory setup and run a Ubuntu 24.04LTS and ROS Jazzy inside a docker container on a Raspberry Pi 5.  Ubuntu 24.04LTS can be installed natively on the the Pi5 but camera support is sadly lacking.  Using a docker on the Pi5 is the simplest way of working around this problem.

## Installation and setup

Simply run the commands as detailed below.

1. If you don't already have docker, install it using:

   ```bash
   cd docker_ros
   ./install_docker.bash
   ```

2. **Log out and log back in or reboot** (required for group changes to take effect)

3. Build the docker image.

   ```bash
   cd docker_ros
   ./build_image.bash
   ```

   Do this just once!

## Day to day use

Once the installation and setup has been completed, you can use the image that has been created. When using ROS, it is often necessary to connect multiple terminal windows to the running container, so we use a two step start up process to allow multiple terminal windows.

1. Create and run a new container using:

   ```bash
   cd docker_ros
   ./start.bash
   ```

2. Attach a terminal session to the running container using:

   ```bash
   cd docker_ros
   ./attach.bash
   ```

   The `attach.bash` script can be called once from each terminal window but you can create many terminal windows.

3. When you have finished with the container, stop it using:

   ```bash
   cd docker_ros
   ./stop.bash
   ```

## Changing the docker image

To change the docker image, modify the `Dockerfile` as required.  Then remove the container and image using:

```bash
cd docker_ros
./rm_container.bash
./rm_image.bash
```

and build the new image using:

```bash
cd docker_ros
./build_image.bash
```

## Changing the ROS version

These scripts were created and tested using ROS Jazzy Perception version.  To update to a newer version, please change the ROS version name in the `Dockerfile`, then remove the container and image and rebuild.
