# syntax=docker/dockerfile:1

FROM osrf/ros:noetic-desktop-full-focal as base
RUN apt-get update \
 && apt-get install -y                 \
       git                             \
       jq                              \
       bash-completion                 \
       command-not-found               \
       ninja-build                     \
       vim                             \
       python3-catkin-tools            \
       wget                            \
       curl                            \
       ros-noetic-ddynamic-reconfigure \
       python3-pip                     \
       python3-imageio                 \
       python3-numpy                   \
       python3-opencv                  \
       python3-netaddr                 \
       ros-noetic-navigation           \
       ros-noetic-teb-local-planner    \
       libceres-dev                    \
       libgflags-dev                   \
       libgoogle-glog-dev              \
       liblua5.2-dev                   \
       python3-sphinx                  \
       stow                            \
 && wget -O /etc/apt/trusted.gpg.d/llvm.asc 'https://apt.llvm.org/llvm-snapshot.gpg.key' \
 && wget -O /etc/apt/trusted.gpg.d/realsense.asc 'https://keyserver.ubuntu.com/pks/lookup?op=hget&search=490918728cb41b2e9f4478a11f27750d' \
 && echo 'deb https://apt.llvm.org/focal/ llvm-toolchain-focal main' > /etc/apt/sources.list.d/llvm.list \
 && echo 'deb https://librealsense.intel.com/Debian/apt-repo focal main' > /etc/apt/sources.list.d/realsense.list \
 && apt-get update \
 && apt-get install -y \
       clangd              \
       librealsense2-dev   \
       librealsense2-utils \
 && apt-get clean
RUN useradd -d /home/sim2real -m -s /bin/bash -U -G sudo,plugdev sim2real \
 && echo 'sim2real ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/nopwd
USER sim2real
ENV QT_X11_NO_MITSHM=1
RUN pip3 install robomaster
ADD --chown=sim2real src/cartographer/scripts/install_abseil.sh /home/sim2real/install_abseil.sh
RUN cd /home/sim2real \
 && ./install_abseil.sh \
 && rm -rf abseil-cpp install_abseil.sh

FROM base as dev
RUN echo 'source ~/workspace/custom.bashrc' >> /home/sim2real/.bashrc \
 && touch /home/sim2real/.sudo_as_admin_successful

FROM base as prod
RUN mkdir /home/sim2real/workspace
ADD --chown=sim2real src /home/sim2real/workspace/src
ADD --chown=sim2real .catkin_tools /home/sim2real/workspace/.catkin_tools
ADD --chown=sim2real build.sh /home/sim2real/workspace/
RUN GENERATE_COMPILE_COMMANDS=false /home/sim2real/workspace/build.sh
ADD entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "--wait", "apriltag_marker_detector", "detection_evaluation.launch"]
