{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-rendering";
  version = "9.1.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-rendering";
    rev = "gz-rendering9";
    hash = "sha256-dqg9Zv3ty8qEI+NWHUmu8TzSw9R4K+lOUNS2ER344gY=";
  };

  buildInputs = with pkgs; [
    freeglut
    freeimage
    glew
    ogre   # Equivalent to libogre-dev in Nixpkgs.
    vulkan-loader # Equivalent to libvulkan-dev in Nixpkgs.
    xorg.libXi
    xorg.libXmu
    util-linux # For uuid support.

    # gz-rendering> -- Looking for GzOGRE - not found
    # ...
    # gz-rendering> -- Looking for GzOGRE2 - not found
    # gz-rendering> -- Skipping vulkan support for component [ogre2]
    # ...
    # gz-rendering> -- Looking for OptiX - not found
    # gz-rendering> CMake Warning at /nix/store/11s1rpm3qx0gg6vjb1zvky4afrja17iv-gz-cmake-4.1.1/share/cmake/gz-cmake4/cmake4/GzConfigureBuild.cmake:59 (message):
    # gz-rendering>    CONFIGURATION WARNINGS:
    # gz-rendering>    -- Skipping component [ogre]: Missing dependency [GzOGRE] (Components: RTShaderSystem, Terrain, Overlay, Paging).
    # gz-rendering>       ^~~~~ Set SKIP_ogre=true in cmake to suppress this warning.
    # gz-rendering> Call Stack (most recent call first):
    # gz-rendering>   CMakeLists.txt:219 (gz_configure_build)
  ];

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    gz-common
    gz-math
    gz-plugin
    gz-utils
  ];

  nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config pkgs.xorg.xvfb];

  meta = with pkgs.lib; {
    maintainers = [ "Ian Chen <ichen@openrobotics.org>" ];
    description = "Gazebo Rendering utilities";
    license = licenses.bsd3;
  };
}

