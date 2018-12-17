# read arguments from command line if we want to run a pytest
# the argument is stored and can be given as an argument to tests

import pytest
import os


def pytest_addoption(parser):
    parser.addoption(
        "--filepath", action="store", default="Please pass absolute log file path as argument", help="absolute path to log file"
    )

# With scope set to module, the fixture function only gets invoked once per module
@pytest.fixture(scope="session")
def filepath(request):
    return request.config.getoption("--filepath")


# This fixture is run automatically, so it does not have to be called explicitely (because of autouse)    
@pytest.fixture(scope="session", autouse=True)
def filecheck(filepath):
    # ensure it is a ulg file
    base, ext = os.path.splitext(filepath)
    if ext.lower() not in (".ulg") or not filepath:
        pytest.exit("passed file is not a .ulg file.")