{ lib, fetchPypi, buildPythonPackage, pythonPackages }:

buildPythonPackage rec {
  pname = "pyulog";
  version = "1.2.0";
  buildInputs = with pythonPackages; [
    numpy
  ];
  src = fetchPypi {
    inherit pname version;
    sha256 = "sha256-DfpSIl8b/AVRKUzRlahjgGAc3QcRIM8qBTNukU/ULsY=";
  };
  pythonImportsCheck = [ "pyulog" ];
  meta = with lib; {
    homepage = "https://github.com/PX4/pyulog";
    description = "Python module & scripts for ULog files.";
    maintainers = with maintainers; [ ];
    license = licenses.bsd3;
  };
}


