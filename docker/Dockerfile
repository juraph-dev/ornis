FROM ros:humble-ros-base

# Add ornis user
RUN useradd -ms /bin/bash ornis
WORKDIR /home/ornis

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

# install ORNIS deps
RUN apt-get install -y build-essential cmake doctest-dev libavdevice-dev libdeflate-dev libgpm-dev \
    libncurses-dev libqrcodegen-dev libswscale-dev libunistring-dev pandoc pkg-config \
    ros-humble-rmw-fastrtps-dynamic-cpp git

user ornis
