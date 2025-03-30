{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-math";
  version = "8.1.1";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-math";
    rev = "gz-math8";
    hash = "sha256-76cQOzk5Pot/RLzFmQyYoL/wVQgjAKAOnjl6e/KFvDk=";
  };

  propagatedBuildInputs = with pkgs; [
    eigen
    python311Packages.pybind11
    gz-cmake
    gz-utils
  ];

  nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config];

  meta = with pkgs.lib; {
    maintainers = [
      "Steve Peters <scpeters@openrobotics.org>"
      "Aditya Pande <aditya050995@gmail.com>"
    ];
    description = "Gazebo Math utilities";
    license = licenses.bsd3;
  };
}

