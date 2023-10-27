#!/bin/bash

sudo xhost local:root

docker build -t ubuntu_docker . --network=host

