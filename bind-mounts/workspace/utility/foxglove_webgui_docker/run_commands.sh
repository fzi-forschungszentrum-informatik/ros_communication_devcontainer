#!/bin/bash

# Run the first container
docker run --name foxglove_server_remote_assistance --rm -p "8080:8080" -v /foxglove-layout.json:/foxglove/default-layout.json ghcr.io/foxglove/studio:latest &

# Construct the URL with the WebSocket IP
URL="http://localhost:8080/?ds=foxglove-websocket&ds.url=ws%3A%2F%2F${WEBSOCKET_IP}%3A8765"

# Run the second container (the browser)
docker run -it --rm --network=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
    -v /dev/shm:/dev/shm \
    -v /foxglove-layout.json:/home/chrome/Downloads/foxglove-layout.json \
    --name browser-for-remote-assistance \
    --security-opt seccomp:unconfined \
    jess/chrome --no-default-browser-check --no-first-run --incognito --no-cache "${URL}"
