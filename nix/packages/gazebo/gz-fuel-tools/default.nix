{ pkgs ? import <nixpkgs> {} }:

pkgs.stdenv.mkDerivation {
  pname = "gz-fuel-tools";
  version = "10.0.1";

  src = pkgs.fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-fuel-tools";
    rev = "gz-fuel-tools10";
    hash = "sha256-J6PlHz2v51RHuG4ThuJjvN3Qha8eYzyrohugJhM6Uqs=";
  };

  # TODO: can be solved differently in cmake?
  postPatch = ''
    # Fix library location path construction
    substituteInPlace src/cmd/CMakeLists.txt \
      --replace 'set(library_location "../../../' \
                'set(library_location "'
  '';

  propagatedBuildInputs = with pkgs; [
    curl.dev
    gflags
    jsoncpp
    libyaml.dev
    libzip
    tinyxml-2
    gz-cmake
    gz-common
    gz-math
    gz-msgs
    gz-tools
    gz-utils
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
  ];

  meta = with pkgs.lib; {
    maintainers = [ "Nate Koenig <natekoenig@gmail.com>" ];
    description = "Gazebo fuel tools utilities";
    license = licenses.bsd3;
  };
}

