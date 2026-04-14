docker stop temp \
&& docker rm temp \
&& docker run -d --name temp --network host --privileged team2-dockerimage tail -f /dev/null \
&& docker cp . temp:/workspace \
&& docker exec -it temp bash \
&& docker rm -f temp