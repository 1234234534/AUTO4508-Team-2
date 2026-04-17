docker stop TOBY || true
docker rm TOBY || true

docker run -d --name TOBY --network host --privileged team2-dockerimage tail -f /dev/null \
&& docker cp . TOBY:/workspace \
&& docker exec -it TOBY bash \
&& docker rm -f TOBY