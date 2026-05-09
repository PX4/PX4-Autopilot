{
  description = "Reproducible build system for PX4";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/197a2b55c4ed24f8b885a5b20b65f426fb6d57ca";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.follows = "nix-ros-overlay/flake-utils";
  };

  outputs = {
    nixpkgs,
    flake-utils,
    nix-ros-overlay,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [nix-ros-overlay.overlays.default];
      };

      pyrosGenmsg = pkgs.python3Packages.buildPythonPackage rec {
        pname = "pyros-genmsg";
        version = "0.5.8";
        pyproject = true;
        build-system = [pkgs.python3Packages.setuptools_79];
        src = pkgs.python3Packages.fetchPypi {
          inherit version;
          pname = "pyros_genmsg";
          sha256 = "0y7l131lc77v0c1rhxza41cxnnxc7acfqzlqf84fdya0kiyv071w";
        };
        doCheck = false;
      };

      pyulog = pkgs.python3Packages.buildPythonPackage rec {
        pname = "pyulog";
        version = "0.8.0";
        pyproject = true;
        build-system = [pkgs.python3Packages.setuptools_79];
        src = pkgs.python3Packages.fetchPypi {
          inherit pname version;
          sha256 = "1ivvhfi9rsrqdk9f06rj0q1d367ngyy0xyc2x9mdwjx3dazwgn45";
        };
        postPatch = ''
          substituteInPlace versioneer.py \
            --replace-fail "configparser.SafeConfigParser()" "configparser.ConfigParser()" \
            --replace-fail "parser.readfp(f)" "parser.read_file(f)"
        '';
        propagatedBuildInputs = [pkgs.python3Packages.numpy];
        doCheck = false;
      };

      flake8Builtins = pkgs.python3Packages.buildPythonPackage rec {
        pname = "flake8-builtins";
        version = "3.0.0";
        pyproject = true;
        build-system = [pkgs.python3Packages.hatchling];
        src = pkgs.python3Packages.fetchPypi {
          inherit version;
          pname = "flake8_builtins";
          sha256 = "0wykv66fg9gzi0w4yyyk5kwvaapz3arlry49yws8d0kbvrcd9d5s";
        };
        propagatedBuildInputs = [pkgs.python3Packages.flake8];
        doCheck = false;
      };

      flake8Comprehensions = pkgs.python3Packages.buildPythonPackage rec {
        pname = "flake8-comprehensions";
        version = "3.17.0";
        pyproject = true;
        build-system = [pkgs.python3Packages.setuptools_79];
        src = pkgs.python3Packages.fetchPypi {
          inherit version;
          pname = "flake8_comprehensions";
          sha256 = "0jhgvkj1n5l2qw7n0dxrf3xxzfjcfarf8acyk6g6qkdzn81a2kxz";
        };
        propagatedBuildInputs = [pkgs.python3Packages.flake8];
        doCheck = false;
      };

      empyCompat = pkgs.python3Packages.buildPythonPackage rec {
        pname = "empy";
        version = "3.3.4";
        pyproject = false;
        src = pkgs.python3Packages.fetchPypi {
          inherit pname version;
          sha256 = "73ac49785b601479df4ea18a7c79bc1304a8a7c34c02b9472cf1206ae88f01b3";
        };
        dontBuild = true;
        installPhase = ''
          runHook preInstall
          mkdir -p "$out/${pkgs.python3.sitePackages}"
          cp em.py "$out/${pkgs.python3.sitePackages}/em.py"
          runHook postInstall
        '';
        doCheck = false;
      };

      empyCompatSitePackages = "${empyCompat}/${pkgs.python3.sitePackages}";

      futureCompat = pkgs.python3Packages.future.overridePythonAttrs (_old: {
        disabled = false;
        doCheck = false;
        pythonImportsCheck = [];
      });

      pythonRequirements = let
        requirementName = line: let
          match = builtins.match "([A-Za-z0-9_.-]+).*" (pkgs.lib.strings.trim line);
        in
          if match == null
          then null
          else builtins.head match;
      in
        builtins.filter (name: name != null) (
          map requirementName (pkgs.lib.splitString "\n" (builtins.readFile ./Tools/setup/requirements.txt))
        );

      pythonRequirementOverrides = {
        empy = empyCompat;
        future = futureCompat;
        pyros-genmsg = pyrosGenmsg;
        pyulog = pyulog;
        setuptools = pkgs.python3Packages.setuptools_79;
      };

      pythonPackageFromRequirement = name:
        pythonRequirementOverrides.${name}
        or pkgs.python3Packages.${pkgs.lib.replaceStrings ["-"] ["_"] name};

      astyle31 = pkgs.astyle.overrideAttrs (old: {
        version = "3.1";

        src = pkgs.fetchurl {
          url = "https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz";
          hash = "sha256-y8xM+ZYpRTS7VvAl1vGZ6/3oGqTCccy9XuHBoxknRdc=";
        };

        cmakeFlags =
          (old.cmakeFlags or [])
          ++ [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          ];

        installPhase = ''
          runHook preInstall
          install -Dm755 astyle $out/bin/astyle
          runHook postInstall
        '';
      });

      pythonPackages =
        (map pythonPackageFromRequirement pythonRequirements)
        ++ [
          pkgs.python3Packages.pip
          pkgs.python3Packages.pynacl
          pkgs.python3Packages.flake8
          pkgs.python3Packages."flake8-blind-except"
          flake8Builtins
          pkgs.python3Packages."flake8-class-newline"
          flake8Comprehensions
          pkgs.python3Packages."flake8-deprecated"
          pkgs.python3Packages."flake8-docstrings"
          pkgs.python3Packages."flake8-import-order"
          pkgs.python3Packages."flake8-quotes"
          pkgs.python3Packages."pytest-repeat"
          pkgs.python3Packages."pytest-rerunfailures"
        ];

      pythonEnv = pkgs.python3.withPackages (_: pythonPackages);

      packages = [
        pkgs.bashInteractive
        pkgs.binutils
        pkgs.ccache
        pkgs.cmake
        pkgs.coreutils
        pkgs.curl
        pkgs.expat
        pkgs.findutils
        pkgs.gawk
        pkgs.gettext
        pkgs.git
        pkgs.gnugrep
        pkgs.gnumake
        pkgs.gnupg
        pkgs.gnused
        pkgs.gnutar
        pkgs.gzip
        pkgs.ninja
        pkgs."pkg-config"
        pkgs.pkgconf
        pkgs.rsync
        pkgs.stdenv.cc
        pkgs.unzip
        pkgs.wget
        pkgs.which
        pkgs.xz
        pkgs.zip
        pythonEnv
        astyle31
        pkgs.ant
        pkgs.asio
        pkgs.boost
        pkgs.cppcheck
        pkgs.doxygen
        pkgs."gcc-arm-embedded-13"
        pkgs.geographiclib
        pkgs.graphviz
        pkgs.lcov
        pkgs.libusb1
        pkgs.shellcheck
        pkgs.texinfo
      ];

      ros = pkgs.rosPackages.kilted;
      rosBuildEnv = ros.buildEnv {
        paths = [
          ros.desktop
          ros.diagnostics
          ros."eigen-stl-containers"
          ros."eigen3-cmake-module"
          ros."geographic-msgs"
          ros."gz-sim-vendor"
          ros."image-transport-plugins"
          ros."rmw-zenoh-cpp"
          ros."ros-gz"
          ros."tf2-geometry-msgs"
        ];
      };

      rosPackages = [
        pkgs.colcon
        pkgs.python3Packages."colcon-ros"
        pkgs.python3Packages.rosdep
        rosBuildEnv
      ];

      shellHook = ''
        export PYTHONNOUSERSITE=1
        export PYTHONPATH=${empyCompatSitePackages}:''${PYTHONPATH:-}
        export ROS_DISTRO=kilted
      '';

      makeApp = pkgs.writeShellApplication {
        name = "px4";
        runtimeInputs = packages;
        text = ''
          export PYTHONNOUSERSITE=1
          export PYTHONPATH=${empyCompatSitePackages}:''${PYTHONPATH:-}
          export ROS_DISTRO=kilted
          if [[ "$1" == "help" ]]; then
            cat <<'EOF'
          Reproducible build system for PX4

          nix develop
            Enter the PX4 dev shell.

          nix run . -- <command>
            Run a PX4 command e.g. `nix run . -- make help` inside the PX4 dev shell.
          EOF
            exit 0
          fi
          exec "$@"
        '';
      };
    in {
      devShells.default = pkgs.mkShell {
        name = "px4";
        packages = packages ++ rosPackages;
        inputsFrom = [rosBuildEnv];
        inherit shellHook;
      };

      apps.default = {
        type = "app";
        program = "${makeApp}/bin/px4";
        meta.description = "Run commands inside the dev shell with PX4 dependencies provided.";
      };
    });

  nixConfig = {
    extra-substituters = ["https://ros.cachix.org"];
    extra-trusted-public-keys = ["ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="];
  };
}
