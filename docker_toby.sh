docker stop TOBY || true
docker rm TOBY || true

docker run -d --name TOBY \
    --network host \
    --privileged \
    --device=/dev/bus/usb \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyAMC0 \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /run/udev:/run/udev \
    team2-dockerimage tail -f /dev/null \
    
docker cp . TOBY:/workspace \
&& docker exec -it TOBY bash \
&& docker rm -f TOBY