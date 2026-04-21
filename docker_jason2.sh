docker stop JASON || true
docker rm JASON || true

docker run -d --name JASON \
    --network host \
    --privileged \
    --device=/dev/bus/usb \
    team2-jason tail -f /dev/null \
    
&& docker cp . JASON:/workspace \
&& docker exec -it JASON bash