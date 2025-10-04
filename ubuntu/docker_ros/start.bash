#!/bin/bash
# Start the docker container.
# set -x

echo "Starting docker container. This can take up to 5 seconds..."

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

docker container inspect ${CONTAINER_NAME} &> /dev/null
if [ $? == 0 ]
then
    # Container exists.
    if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} )" == "running" ]
    then
        # Container is running.
        echo "Container '${CONTAINER_NAME}' is already running."
    else
        # Container exists but is not running.
        docker container start ${CONTAINER_NAME} &> /dev/null
        echo "Container '${CONTAINER_NAME}' started."
    fi
else
    # Container does not exist.
    mkdir -p ${WORKSPACE_DIR}

    # Setup X window for the container to use.
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]
    then
        xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
        if [ ! -z "$xauth_list" ]
        then
            echo $xauth_list | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi

    docker container run \
        --detach \
        --tty \
        --net=host \
        --device /dev/media0:/dev/media0 \
        --device /dev/media1:/dev/media1 \
        --device /dev/media2:/dev/media2 \
        --device /dev/media3:/dev/media3 \
        --device /dev/video0:/dev/video0 \
        --device /dev/video1:/dev/video1 \
        --device /dev/video2:/dev/video2 \
        --device /dev/video3:/dev/video3 \
        --device /dev/dri:/dev/dri \
        --volume /lib/firmware:/lib/firmware \
        --volume /run/udev:/run/udev:ro \
        --name ${CONTAINER_NAME} \
        --volume ${WORKSPACE_DIR}:/home/ubuntu/ws \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        ${IMAGE_NAME}:${IMAGE_TAG} &> /dev/null
    if [ $? == 0 ]
    then
        echo "Container '${CONTAINER_NAME}' running."
    else
        echo "Container '${CONTAINER_NAME}' failed."
    fi
fi
