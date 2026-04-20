docker stop JASON || true
docker rm JASON || true

docker run -d --name JASON --network host --privileged team2-jason tail -f /dev/null \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
&& docker cp . JASON:/workspace \
&& docker exec -it JASON bash \
&& docker rm -f JASON