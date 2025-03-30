{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "sdformat";
  version = "15.2.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "sdformat";
    rev = "sdf15";
    hash = "sha256-UG9tNrgHsqIraSxkWbh/tJmvQ0sUy6+zwUTFdMo0kls=";
  };

  buildInputs = with pkgs; [
    python311Packages.pybind11
    tinyxml-2
    urdfdom
  ];

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    gz-math
    gz-utils
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
    libxml2
    python3Packages.psutil
    python3Packages.pytest
  ];

  meta = with pkgs.lib; {
    maintainers = [
      "Addisu Z. Taddese <addisu@openrobotics.org>"
      "Steve Peters <scpeters@openrobotics.org>"
    ];
    description = "Gazebo SDFormat utilities";
    license = licenses.bsd3;
  };
}

