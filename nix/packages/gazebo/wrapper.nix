# Function to create the gz wrapper
pkgs: { gz-tools, plugins }:
let
  gzConfigPath = pkgs.lib.concatStringsSep ":" ([
    "${gz-tools}/share/gz"
  ] ++ map (p: "${p}/share/gz") plugins);
in
pkgs.writeShellScriptBin "gz" ''
  export GZ_CONFIG_PATH="${gzConfigPath}"
  exec "${gz-tools}/bin/gz" "$@"
''
