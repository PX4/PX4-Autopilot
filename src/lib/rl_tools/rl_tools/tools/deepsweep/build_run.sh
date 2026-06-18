docker build -t rltools/sweep .
docker push rltools/sweep
docker run -itd --name deepsweep --restart unless-stopped -p 13338:13338 --volume deepsweep:/jobs rltools/sweep