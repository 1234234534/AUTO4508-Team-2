docker stop WINTON || true
docker rm WINTON || true

xhost +local:docker

docker run -d --name WINTON \
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
    
docker cp . WINTON:/workspace \
&& docker exec -it WINTON bash \
&& docker rm -f WINTON