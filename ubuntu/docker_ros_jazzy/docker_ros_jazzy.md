# ROS Jazzy on a Pi 5

The scripts in this directory setup and run a Ubuntu 24.04LTS and ROS Jazzy inside a docker container on a Raspberry Pi 5.  Ubuntu 24.04LTS can be installed natively on the the Pi5 but camera support is sadly lacking.  Using a docker on the Pi5 is the simplest way of making it work.

## Method

Simply run the scripts as detailed below.

1. **Run Part 1** (installs Docker and configures user groups):

   ```bash
   ./setup1.bash
   ```

2. **Log out and log back in** (required for group changes to take effect)

3. **Run Part 2** (builds containers, sets up development environment):

   ```bash
   ./setup2.bash
   ```

This two-part setup ensures proper configuration of Docker permissions and provides a complete development environment with:

- Docker and containerization support
- ROS 2 development container
- X11 forwarding for GUI applications
- Camera support and testing
- Development directory structure
- Example configurations and launch files

## Acknowledgements

These scripts were written with help from ChatGPT 5 mini.  However, the results did not work well, so the scripts have been hand crafted so they actually work.
