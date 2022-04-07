# syntax=docker/dockerfile:1

FROM osrf/ros:noetic-desktop-full-focal as base
RUN apt-get update \
 && apt-get install -y       \
       git                  \
       jq                   \
       bash-completion      \
       command-not-found    \
       ninja-build          \
       vim                  \
       python3-catkin-tools \
       wget                 \
       curl                 \
 && wget -O /etc/apt/trusted.gpg.d/llvm.asc 'https://apt.llvm.org/llvm-snapshot.gpg.key' \
 && echo 'deb https://apt.llvm.org/focal/ llvm-toolchain-focal main' > /etc/apt/sources.list.d/llvm.list \
 && apt-get update \
 && apt-get install -y \
       clangd \
 && apt-get clean
RUN useradd -d /home/sim2real -m -s /bin/bash -U -G sudo sim2real \
 && echo 'sim2real ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/nopwd
USER sim2real
ENV QT_X11_NO_MITSHM=1

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
CMD ["roslaunch", "rmus_bringup", "test.launch"]
