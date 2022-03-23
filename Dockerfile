# syntax=docker/dockerfile:1

FROM rmus2022/client:v1.2.0 as base
RUN apt-get update
RUN apt-get install -y jq
RUN usermod -s /bin/bash sim2real
USER sim2real
ENV QT_X11_NO_MITSHM=1
RUN cp /etc/skel/.bashrc /home/sim2real/.bashrc

FROM base as dev
ADD --chown=sim2real custom.bashrc /home/sim2real/
RUN echo "source ~/custom.bashrc" >> /home/sim2real/.bashrc

FROM base as prod
RUN mkdir /home/sim2real/workspace
ADD --chown=sim2real src /home/sim2real/workspace/src
ADD --chown=sim2real build.sh /home/sim2real/workspace/
RUN BUILD_RELEASE=true /home/sim2real/workspace/build.sh
ADD client_start.sh /
CMD /client_start.sh
