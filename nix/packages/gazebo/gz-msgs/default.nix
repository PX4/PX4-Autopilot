{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation rec {
  pname = "gz-msgs";
  version = "11.0.2";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-msgs";
    rev = "gz-msgs11";
    hash = "sha256-UBEhKkA2KBahP+9K4m7A1WkrguYOYHIjwuR3Chh1M5g=";
  };

  propagatedBuildInputs = with pkgs; [
    python3
    python3Packages.protobuf
    tinyxml-2
    gz-cmake
    gz-math
    gz-tools
  ];

  nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config];

  postInstall = ''
    mkdir -p $out/bin
    ln -s $out/libexec/gz/msgs${pkgs.lib.versions.major version}/gz-msgs $out/bin/
  '';

  meta = with pkgs.lib; {
    maintainers = [ "Carlos Agüero <caguero@openrobotics.org>" ];
    description = "Gazebo Messages utilities";
    license = licenses.bsd3;
    mainProgram = "gz-msgs";
  };
}
