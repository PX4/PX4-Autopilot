{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation rec {
  pname = "gz-common";
  version = "6.0.2";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-common";
    rev = "gz-common6";
    hash = "sha256-Yx4c7tKauRNI11hOPx4oOQQNJtjKM700NgQP4w11UgU=";
  };

  propagatedBuildInputs = with pkgs; [
    assimp
    ffmpeg
    freeimage
    gdal
    spdlog
    tinyxml-2
    util-linux # For uuid
    gz-cmake
    gz-math
    gz-utils
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
  ];

  meta = with pkgs.lib; {
    maintainers = [ "Nate Koenig <natekoenig@gmail.com>" ];
    description = "Gazebo common utilities";
    license = licenses.bsd3;
  };
}

