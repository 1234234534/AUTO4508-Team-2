docker stop WINTON || true
docker rm WINTON || true

docker run -d --name WINTON \
    --network host \
    --privileged \
    --device=/dev/bus/usb \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyAMC0 \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /run/udev:/run/udev \
    team2-dockerimage tail -f /dev/null \
    
docker cp . WINTON:/workspace \
&& docker exec -it WINTON bash \
&& docker rm -f WINTON