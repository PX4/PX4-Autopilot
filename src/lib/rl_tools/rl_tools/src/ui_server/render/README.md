## Get `data.json` and `ui.js`
```
gunzip -c experiments/2025-05-23_13-40-39/1c19b2c_zoo_environment_algorithm/l2f_sac/0001/steps/000000002000000/trajectories.json.gz > data.json
```
```
cp experiments/2025-05-23_13-40-39/1c19b2c_zoo_environment_algorithm/l2f_sac/0001/ui.esm.js ui.js
```


## API
### Trajectory
```
jq '{parameters: .[0].parameters, trajectory: .[0].trajectory[0:10]}' data.json \
  | curl -X POST http://localhost:13339/render_trajectory \
    -F "data=@-;type=application/json" \
    -F "ui=@ui.js" \
    -F "width=2000" \
    -F "height=2000" \
    --output response.zip
```
### Single Frame
```
jq '{parameters: .[0].parameters, step: .[0].trajectory[0]}' data.json \
  | curl -X POST http://localhost:13339/render \
    -F "data=@-;type=application/json" \
    -F "ui=@ui.js" \
    -F "width=2000" \
    -F "height=2000" \
    --output response.png
```


## Make into video:
```
ffmpeg -framerate 100 -i frame_%05d.png -c:v libx264 -pix_fmt yuv420p -crf 18 output.mp4
```


## Docker:
```
docker run -it --rm -v $(pwd):/mnt -w /mnt ghcr.io/puppeteer/puppeteer:latest node puppeteer.js
```