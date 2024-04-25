#!/bin/bash

device_options=""

if [ -e /dev/ttyUSB0 ]; then
    device_options+=" --device=/dev/ttyUSB0"
fi

if [ -e /dev/ttyUSB1 ]; then
    device_options+=" --device=/dev/ttyUSB1"
fi

if [ -e /dev/ttyUSB2 ]; then
    device_options+=" --device=/dev/ttyUSB2"
fi

if [ -e /dev/ttyUSB3 ]; then
    device_options+=" --device=/dev/ttyUSB3"
fi

if [ -e /dev/ttyUSB4 ]; then
    device_options+=" --device=/dev/ttyUSB4"
fi

# 建構完整指令
docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network host $device_options --env-file ./.env --hostname pros-ai-image ghcr.io/otischung/pros_ai_image:latest /bin/bash
