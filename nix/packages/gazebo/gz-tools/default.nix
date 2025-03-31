{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-tools";
  version = "2.0.2";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-tools";
    rev = "gz-tools2";
    hash = "sha256-o9Br6HuowGiHRqGET8xc5tqTVYPqTHOpVQnpH3UQP/w=";
  };

  propagatedBuildInputs = with pkgs; [
    gz-cmake
    ruby    # Provides Ruby runtime.
    rubocop
  ];

  nativeBuildInputs = with pkgs; [
    python3
    cmake
    pkg-config
    makeWrapper
  ];

  # Set LD_LIBRARY_PATH to include the library directory (libgz-tools2-backward.so discovery)
  postFixup = ''
    wrapProgram $out/bin/gz \
      --prefix LD_LIBRARY_PATH : "$out/lib"
  '';

  meta = with pkgs.lib; {
    maintainers = [ "Carlos Agüero <caguero@openrobotics.org>" ];
    description = "Gazebo Tools utilities";
    license = licenses.bsd3;
  };
}

