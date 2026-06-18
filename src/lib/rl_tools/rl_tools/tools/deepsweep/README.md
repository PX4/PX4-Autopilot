
## Create Job
```
cat <<EOF > tasks.ndjson                                                          
{"seed": 0}
{"seed": 1}
{"seed": 2}
EOF
```

```
curl -X POST --data-binary @tasks.ndjson http://localhost:13338/jobs/cartpole_sweep
```

From pipe:
```
cat tasks.ndjson | curl -X POST --data-binary @- http://localhost:13338/jobs/cartpole_sweep
```


## Take Task
```
curl -X POST http://localhost:13338/jobs/cartpole_sweep/tasks
```

## Submit Result
```
curl -X POST -H 'Content-Type: application/json' --data '{"reward": 495.0}' http://localhost:13338/jobs/cartpole_sweep/tasks/1
```

## Reset In-Progress

```
curl -X POST http://localhost:13338/jobs/cartpole_sweep/reset
```


## Run on Server (behind VPN)
```
docker run -itd --restart unless-stopped --rm --name sweep -p 10.8.0.1:13338:13338 rltools/sweep
```