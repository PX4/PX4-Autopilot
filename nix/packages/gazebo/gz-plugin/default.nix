{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation rec {
  pname = "gz-plugin";
  version = "3.0.1";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-plugin";
    rev = "gz-plugin3";
    hash = "sha256-Qn4G6hLFEaVQO02E0qL1yqkdGDNLz/a1dpy+jiYtNbY=";
  };

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    gz-tools
    gz-utils
  ];

  nativeBuildInputs = with pkgs; [
    python3
    cmake
    pkg-config
  ];

  postInstall = ''
    mkdir -p $out/bin
    ln -s $out/libexec/gz/plugin${pkgs.lib.versions.major version}/gz-plugin $out/bin/
  '';

  meta = with pkgs.lib; {
    maintainers = [ "Alejandro Hernández Cordero <ahcorde@gmail.com>" ];
    description = "Gazebo Plugin utilities";
    license = licenses.bsd3;
    mainProgram = "gz-plugin";
  };
}
