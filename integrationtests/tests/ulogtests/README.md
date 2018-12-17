# px4ulogtests
Testing scripts to test ulogs for px4-specific implementations. 

## Installation
To prevent any conflict with the system python version, it is suggested to use a virtual enrionment with python version 3.6 and higher. Otherwise, python 3.6 and higher must be the python system version.
If you don't have 3.6 installed on your machinge, you can follow this [tutorial](http://ubuntuhandbook.org/index.php/2017/07/install-python-3-6-1-in-ubuntu-16-04-lts/).

### virtualenvwrapper

First install virtualenv:
```bash
sudo apt install virtualenv
```

Install virtualenvrapper: this will install `virtualenvwrapper.sh` in `~/.local/bin`
```bash
pip install virtualenvwrapper
```

Create a virtual environement directory
```bash
mkdir ~/.virtualenvs
```

Add virtual envrionment working-folder to bashrc and source virtualenvwrapper:
```bash
export WORKON_HOME=$HOME/.virtualenvs
source $HOME/.local/bin/virtualenvwrapper.sh
```

Open new terminal or source bashrc:
```bash
source ~/.bashrc
```

Create a virtual environment with python version 3 and no site packages included (python3 must be installed)
```bash
mkvirtualenv --python=python3 --no-site-packages [name-of-new-env]
```

You now created a new virtual environment with name [name-of-new-env].

To enter [name-of-new-env]:
```bash
workon [name-of-new-env]
```

To exit [name-of-new-env]:
```bash
deactivate
```

### test a ulog file 
To test a log file, pass its absolute path as a command line argument (see example below)

All the tests are in the `tests` directory. Tests should be split into tests that apply for any ulog file and tests that are coupled to a specific simulated test.

To run general tests:
```bash
py.test -s tests/test_general.py --filepath=absolute/path/to/logfile.ulg
```

Using -s allows you to see print statements for the skipped tests

With the current default log, the test for tilt will fail because the vehicle flipped during the actual flight. 


