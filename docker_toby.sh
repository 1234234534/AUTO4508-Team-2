docker stop TOBY || true
docker rm TOBY || true

xhost +local:docker

docker run -d --name TOBY \
    --network host \
    --privileged \
    --device=/dev/bus/usb \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyAMC0 \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /run/udev:/run/udev \
    team2-dockerimage tail -f /dev/null \
    
docker cp . TOBY:/workspace \
&& docker exec -it TOBY bash \
&& docker rm -f TOBY