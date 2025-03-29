# HOW TO USE:
# 1. nix develop --profile profiles/dev --command $SHELL
# 2. done!
#
# if you want more logs during build, add `-L` to `nix develop`.
{
  description = "PX4 Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs = { self, nixpkgs }: let
    system = "x86_64-linux";
    pkgs = import nixpkgs { inherit system; };
  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [
        # General development dependencies
        pkgs.astyle
        pkgs.buildPackages.gcc
        pkgs.ccache
        pkgs.cmake
        pkgs.cppcheck
        pkgs.file
        pkgs.gdb
        pkgs.git
        pkgs.lcov
        pkgs.libxml2
        pkgs.makeWrapper
        pkgs.ninja
        (pkgs.python3.withPackages (p: [
          p.argcomplete
          p.cerberus
          p.coverage
          (p.callPackage ./nix/packages/empy {})
          p.future
          p.jinja2
          p.jsonschema
          p.kconfiglib
          p.lxml
          (p.matplotlib.override { enableQt = false; }) # Disable Qt support for simplicity.
          p.numpy
          p.nunavut
          p.packaging
          p.pandas
          p.pkgconfig
          p.psutil
          p.pygments
          p.wheel
          p.pymavlink
          (p.callPackage ./nix/packages/pyros-genmsg {})
          p.pyserial
          (p.callPackage ./nix/packages/pyulog {})
          p.pyyaml
          p.requests
          p.setuptools
          p.six
          p.toml
          p.sympy # Sympy is included with version >=1.10.1 in nixpkgs.
        ]))
        pkgs.rsync
        pkgs.shellcheck
        pkgs.unzip
        pkgs.zip

        # NuttX toolchain dependencies (conditional installation can be added later)
        pkgs.automake
        pkgs.bison
        pkgs.flex
        pkgs.gcc-arm-embedded # arm-none-eabi-gcc equivalent in Nixpkgs
        # pkgs.gdb-multiarch
        pkgs.genromfs
        pkgs.gettext
        pkgs.gperf
        pkgs.kconfig-frontends
        pkgs.libelf # .dev
        pkgs.expat.dev
        pkgs.gmp.dev
        pkgs.isl
        pkgs.libmpc
        pkgs.mpfr.dev
        pkgs.ncurses.dev # libncurses-dev equivalent in Nixpkgs

        # Simulation dependencies (Gazebo and related tools)
        pkgs.eigen # libeigen3-dev equivalent in Nixpkgs
      ];

      shellHook = ''
      echo "Welcome to the PX4 development environment!"
      '';
    };
  };
}
