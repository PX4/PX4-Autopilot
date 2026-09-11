variable "SIMULATOR" {
  default = "sih"
}

variable "VERSION" {
  default = "dev"
}

variable "ARCH" {
  default = split("/", BAKE_LOCAL_PLATFORM)[1]
}

variable "CACHE_GHA" {
  default = false
}

variable "REPOSITORY" {
  default = SIMULATOR == "sih" ? "px4-sitl" : "px4-sitl-gazebo"
}

function "image_tags" {
  params = [repository]
  result = flatten([
    for registry in ["px4io", "ghcr.io/px4"] : [
      for tag in [VERSION, "latest"] : "${registry}/${repository}:${tag}-${ARCH}"
    ]
  ])
}

group "default" {
  targets = ["sitl", "ros2"]
}

target "_common" {
  context    = "docker-context"
  platforms  = ["linux/${ARCH}"]
  cache-from = CACHE_GHA ? ["type=gha,scope=sitl-${SIMULATOR}-${ARCH}"] : []
}

target "sitl" {
  inherits   = ["_common"]
  dockerfile = "Dockerfile.${SIMULATOR}"
  tags       = image_tags(REPOSITORY)
}

target "ros2" {
  inherits   = ["_common"]
  dockerfile = "Dockerfile.ros2"
  tags       = image_tags("${REPOSITORY}-ros2")
  contexts = {
    sitl = "target:sitl"
  }

  # The child graph includes the parent, so one export caches both images.
  cache-to = CACHE_GHA ? ["type=gha,mode=max,scope=sitl-${SIMULATOR}-${ARCH}"] : []
}

# Published independently of the packaged SITL images.
target "ros2-dev" {
  context    = "docker-context"
  platforms  = ["linux/${ARCH}"]
  dockerfile = "Dockerfile.ros2"
  target     = "ros2-dev"
  tags       = ["px4io/px4-dev-ros2:main-jazzy-${ARCH}", "ghcr.io/px4/px4-dev-ros2:main-jazzy-${ARCH}"]
  cache-from = CACHE_GHA ? ["type=gha,scope=ros2-dev-${ARCH}"] : []
  cache-to   = CACHE_GHA ? ["type=gha,mode=max,scope=ros2-dev-${ARCH}"] : []
}
