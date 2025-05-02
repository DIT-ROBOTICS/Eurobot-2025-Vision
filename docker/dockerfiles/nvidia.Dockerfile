ARG ARCH
ARG BASE_IMAGE
###### Ultrayltics Module ######
FROM --platform=${ARCH} ${BASE_IMAGE} AS ultralytics-ros
ARG USER
ARG USER_UID
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive 
# Copy binaries from ros humble base stage
COPY ../scripts/build/ /tmp/
RUN sh /tmp/install_depend.sh
RUN apt-get update && apt-get install -y \
    libblas3 liblapack3 \
    && apt-get clean -y && rm -rf /var/lib/apt/list/* && \
    sh /tmp/setup_user.sh $USER $USER_UID $USER_GID && \
    mkdir -p /home/$USER/vision-ws/src && \
    sh /tmp/rosdep_init.sh $USER
USER $USER
RUN pip install --no-cache-dir ultralytics
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" ]