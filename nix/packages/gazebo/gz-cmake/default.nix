{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-cmake";
  version = "4.1.1";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-cmake";
    rev = "gz-cmake4";
    hash = "sha256-OL8QMRt++UUSMxKaQmYZDgAOPxBfJZcpYzXNPyYMzNo=";
  };

  nativeBuildInputs = with pkgs; [
    python3
    cmake
    pkg-config
  ];

  meta = with pkgs.lib; {
    maintainers = [ "Steve Peters <scpeters@openrobotics.org>" ];
    description = "Gazebo cmake utilities";
    license = licenses.bsd3;
  };
}

