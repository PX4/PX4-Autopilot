.PHONY: docker_build docker_push all

DATE := $(shell date +%Y-%m-%d)

docker_build:
	docker build -f Dockerfile -t px4-dev:${DATE} .

docker_push: build
        docker push px4io/px4-dev:${DATE}

all: docker_build
