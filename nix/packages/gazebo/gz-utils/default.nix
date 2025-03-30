{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-utils";
  version = "3.1.1";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-utils";
    rev = "gz-utils3";
    hash = "sha256-wZvl+fi5Wk/x+Ss3CHvstHdUc8P67yMA8N7tsmd3Eug=";
  };

  propagatedBuildInputs = with pkgs; [
    spdlog
    gz-cmake
  ];

  nativeBuildInputs = with pkgs; [
    python3
    cmake
    pkg-config
  ];

  meta = with pkgs.lib; {
    maintainers = [ "Addizu Z. Taddese <addisu@openrobotics.org>" ];
    description = "Gazebo Utilities";
    license = licenses.bsd3;
  };
}

