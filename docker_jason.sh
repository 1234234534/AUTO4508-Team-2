docker stop JASON || true
docker rm JASON || true

xhost +local:docker

docker run -d --name JASON \
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
    -v "/media/team2/UBUNTU 24_0":/images \
    team2-dockerimage tail -f /dev/null \
    
docker cp . JASON:/workspace \
&& docker exec -it JASON bash \
&& docker rm -f JASON