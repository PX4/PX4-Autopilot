# Function to create the gz wrapper with Qt support
pkgs: { gz-tools, plugins }:
let
  # Combine GZ_CONFIG_PATH from gz-tools and plugins
  gzConfigPath = pkgs.lib.concatStringsSep ":" ([
    "${gz-tools}/share/gz"
  ] ++ map (p: "${p}/share/gz") plugins);

  # Combine all dependencies for Qt path resolution
  allDeps = [ gz-tools ] ++ plugins;
in
pkgs.stdenv.mkDerivation {
  name = "gz-wrapper";
  buildInputs = allDeps;
  nativeBuildInputs = [ pkgs.makeWrapper pkgs.qt5.wrapQtAppsHook ];

  # No need to unpack anything
  dontUnpack = true;
  dontConfigure = true;
  dontBuild = true;

  installPhase = ''
    mkdir -p $out/bin
    
    # Create initial wrapper for GZ_CONFIG_PATH
    makeWrapper ${gz-tools}/bin/gz $out/bin/gz \
      --prefix GZ_CONFIG_PATH : "${gzConfigPath}"
    
    # Apply Qt wrapping to include plugin dependencies
    # wrapQtApp $out/bin/gz
  '';
  
  # Propagate Qt dependencies from all plugins
  propagatedBuildInputs = allDeps;
}
