{ lib, fetchPypi, buildPythonPackage }:

buildPythonPackage rec {
  pname = "pyros_genmsg";
  version = "0.5.8";
  src = fetchPypi {
    inherit pname version;
    sha256 = "sha256-PBywfZxA+eYIcph+7Jg6rFvbWSDqd5gDA/scRsMI9Hg=";
  };
  pythonImportsCheck = [ "genmsg" ];
  meta = with lib; {
    homepage = "https://github.com/ros/genmsg";
    description = "Standalone Python library for generating ROS message and service data structures for various languages.";
    maintainers = with maintainers; [ ];
    license = licenses.lgpl21Only;
  };
}


