docker stop JASON || true
docker rm JASON || true

docker run -d --name JASON \
    --network host \
    --privileged \
    --device=/dev/bus/usb \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyAMC0 \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /run/udev:/run/udev \
    team2-dockerimage tail -f /dev/null \
    
docker cp . JASON:/workspace \
&& docker exec -it JASON bash \
&& docker rm -f JASON