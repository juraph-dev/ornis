#!/usr/bin/env sh

docker container stop ornis_docker
docker container rm ornis_docker
docker run --network=host --ipc=host --pid=host --privileged --ulimit nofile=1024:2056 --env="DISPLAY" -it -v /dev/shm:/dev/shm -v ~/ros2_ws/ornis_ws:/home/ornis/ornis_ws:Z --name ornis_docker ornis_docker
