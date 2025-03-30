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
    pkgs = import nixpkgs {
      inherit system;
      overlays = [
        (import ./nix/packages/gazebo/overlays.nix)
      ];
      config = {
        permittedInsecurePackages = [
          # Known issues:
          #  - CVE-2024-31570
          #  - CVE-2024-28584
          #  - CVE-2024-28583
          #  - CVE-2024-28582
          #  - CVE-2024-28581
          #  - CVE-2024-28580
          #  - CVE-2024-28579
          #  - CVE-2024-28578
          #  - CVE-2024-28577
          #  - CVE-2024-28576
          #  - CVE-2024-28575
          #  - CVE-2024-28574
          #  - CVE-2024-28573
          #  - CVE-2024-28572
          #  - CVE-2024-28571
          #  - CVE-2024-28570
          #  - CVE-2024-28569
          #  - CVE-2024-28568
          #  - CVE-2024-28567
          #  - CVE-2024-28566
          #  - CVE-2024-28565
          #  - CVE-2024-28564
          #  - CVE-2024-28563
          #  - CVE-2024-28562
          #  - CVE-2024-9029
          #  - CVE-2023-47996
          #  - CVE-2023-47994
          #  - CVE-2023-47993
          #  - CVE-2023-47992
          #  - CVE-2021-40265
          #  - CVE-2021-40264
          #  - CVE-2021-40262
          #  - CVE-2020-24294
          #  - CVE-2020-21426
          #  - CVE-2019-12214
          #  - CVE-2019-12212
          "freeimage-3.18.0-unstable-2024-04-18"
        ];
      };
    };

    # Default plugins (customize this list as needed)
    gzPlugins = [
      pkgs.gz-sim
      # Add other plugins here
    ];
    
    mkGzWrapper = import ./nix/packages/gazebo/wrapper.nix pkgs;
  in {

    apps.${system}.gz = {
      type = "app";
      program = "${self.packages.${system}.gz}/bin/gz";
      description = "Gazebo Simulation";
    };

    packages.${system} = {
      gz = mkGzWrapper {
        inherit (pkgs) gz-tools;
        plugins = gzPlugins;
      };

      # direct build targets
      gz-cmake = pkgs.gz-cmake;
      gz-common = pkgs.gz-common;
      gz-fuel-tools = pkgs.gz-fuel-tools;
      gz-gui = pkgs.gz-gui;
      gz-launch = pkgs.gz-launch;
      gz-math = pkgs.gz-math;
      gz-msgs = pkgs.gz-msgs;
      gz-physics = pkgs.gz-physics;
      gz-plugin = pkgs.gz-plugin;
      gz-rendering = pkgs.gz-rendering;
      gz-sensors = pkgs.gz-sensors;
      gz-sim = pkgs.gz-sim;
      gz-tools = pkgs.gz-tools;
      gz-transport = pkgs.gz-transport;
      gz-utils = pkgs.gz-utils;
      sdformat = pkgs.sdformat;
    };

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

        # Gazebo Simulation - configure available modules above where gz is defined
        self.packages.${system}.gz        
      ];

      shellHook = ''
      echo "Welcome to the PX4 development environment!"
      '';
    };
  };
}
