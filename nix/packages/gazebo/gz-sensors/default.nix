{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-sensors";
  version = "9.1.0";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-sensors";
    rev = "gz-sensors9";
    hash = "sha256-5V8QYI5swYGdDrqN56R96DIdD8Erl7IWgLBs8fj8HKc=";
  };

  buildInputs = with pkgs; [
    zlib
    cppzmq
    libGL
  ];

  propagatedBuildInputs = with pkgs; [
    gz-common
    gz-math
    gz-msgs
    gz-rendering
    gz-tools
    gz-transport
    sdformat
  ];

  nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config pkgs.xorg.xvfb];

  meta = with pkgs.lib; {
    maintainers = [ "Ian Chen <ichen@openrobotics.org>" ];
    description = "Gazebo Sensors utilities";
    license = licenses.bsd3;
  };
}

