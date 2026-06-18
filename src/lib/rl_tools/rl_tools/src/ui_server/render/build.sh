docker build -t rltools/render -f Dockerfile $@ . && docker run -it --rm -p 13339:13339 rltools/render
docker tag rltools/render:latest rltools/render