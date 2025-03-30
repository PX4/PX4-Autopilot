{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation rec {
  pname = "gz-sim";
  version = "10.0.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-sim";
    rev = "main";
    hash = "sha256-tKjEJ2iGCE26lG0Y6xDRcUrqbUt0CVR01zMiFOAEca0=";
  };

  postPatch = ''
    # Fix library location path construction
    substituteInPlace src/cmd/CMakeLists.txt \
      --replace 'set(library_location "../../../' \
                'set(library_location "'
  '';

  buildInputs = with pkgs; [
    freeglut
    freeimage
    glew
    cppzmq
    xorg.libXi.dev
    xorg.libXmu.dev
    protobufc.dev
    python311Packages.pybind11
    qt5.qtbase
    qt5.qtquickcontrols2
    tinyxml-2
    util-linux # For uuid support.
    libsodium
  ];

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    gz-common
    gz-fuel-tools
    gz-gui
    gz-math
    gz-msgs
    gz-physics
    gz-plugin
    gz-rendering
    gz-sensors
    gz-tools
    gz-transport
    gz-utils
    sdformat
  ];

  nativeBuildInputs =  with pkgs; [
    cmake
    pkg-config
    xorg.xvfb
    qt5.wrapQtAppsHook
  ];

  postInstall = ''
    mkdir -p $out/bin
    ln -s $out/libexec/gz/sim${pkgs.lib.versions.major version}/gz-sim-model $out/bin/
  '';

  meta = with pkgs.lib; {
    maintainers = [ "Michael Carroll <mjcarroll@intrinsic.ai>" ];
    description = "Gazebo Simulator utilities";
    license = licenses.bsd3;
    mainProgram = "gz-sim-model";
  };
}

