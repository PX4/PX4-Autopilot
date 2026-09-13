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

variable "ROS_DISTRO" {
  default = "jazzy"
  validation {
    condition     = contains(["humble", "jazzy"], ROS_DISTRO)
    error_message = "ROS_DISTRO must be humble or jazzy."
  }
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
  args = {
    ROS_DISTRO = ROS_DISTRO
  }
  tags       = ["px4io/px4-dev-ros2:main-${ROS_DISTRO}-${ARCH}", "ghcr.io/px4/px4-dev-ros2:main-${ROS_DISTRO}-${ARCH}"]
  cache-from = CACHE_GHA ? ["type=gha,scope=ros2-dev-${ROS_DISTRO}-${ARCH}"] : []
  cache-to   = CACHE_GHA ? ["type=gha,mode=max,scope=ros2-dev-${ROS_DISTRO}-${ARCH}"] : []
}
