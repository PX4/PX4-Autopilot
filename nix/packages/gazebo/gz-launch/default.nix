{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation rec {
  pname = "gz-launch";
  version = "9.0.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-launch";
    rev = "main";
    hash = "sha256-Mw2kILEUEfzbglegGFi/UuKBcmxbydXN1SDb3AJLV44=";
  };

  buildInputs = with pkgs; [
    binutils          # Provides libbfd
    elfutils          # Provides libdw and libelf
    libdwarf          # DWARF debugging library
    gflags
    cppzmq
    libwebsockets.dev
    xorg.libXi
    xorg.libXmu
    yaml-cpp
    tinyxml-2
    util-linux # For uuid support
    qt5.qtbase
    qt5.qtquickcontrols2
  ];

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    gz-common
    gz-math
    gz-msgs
    gz-plugin
    gz-sim
    gz-tools
    gz-transport
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
    xorg.xvfb
    qt5.wrapQtAppsHook
  ];

  postInstall = ''
    mkdir -p $out/bin
    ln -s $out/lib/gz/launch${pkgs.lib.versions.major version}/gz-launch $out/bin/
  '';


  meta = with pkgs.lib; {
    maintainers = [ "Nate Koenig <natekoenig@gmail.com>" ];
    description = "Gazebo Launch utilities";
    license = licenses.bsd3;
    mainProgram = "gz-launch";
  };
}

