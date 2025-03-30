{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-gui";
  version = "10.0.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-gui";
    rev = "main";
    hash = "sha256-JJNP0pImmDwWwIADDjb2HzQlGVznrQoqnifQeAXShHM=";
  };

  buildInputs = with pkgs; [
    cppzmq
    protobuf
    tinyxml-2
    qt5.qtbase
    qt5.qtquickcontrols2
    qt5.qtcharts
    qt5.qtgraphicaleffects
    qt5.qtlocation
    qt5.qtpositioning
    qt5.qtdeclarative
      # qt5.qtlabs.folderlistmodel
      # qt5.qtlabs.platform
      # qt5.qtlabs.settings
  ];

  cmakeFlags = [
    "-DCMAKE_SKIP_BUILD_RPATH=ON"
    "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
    "-DCMAKE_INSTALL_RPATH=\$ORIGIN/../lib"
  ];


  propagatedBuildInputs = with pkgs; [
    gz-common
    gz-math
    gz-msgs
    gz-plugin
    gz-rendering
    gz-tools
    gz-transport
    gz-utils
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
    xorg.xvfb
    qt5.wrapQtAppsHook
  ];

  meta = with pkgs.lib; {
    maintainers = [ "Jenn Nguyen <jennuine@google.com>" ];
    description = "Gazebo GUI utilities";
    license = licenses.bsd3;
  };
}

