#!/bin/bash

# Wait until the topic is available and has a known type
until ros2 topic info /chatter | grep -q "Type:"; do
    echo "[INFO] Waiting for /chatter to become available..."
    sleep 1
done

# Now start echoing the topic
exec ros2 topic echo /chatter
