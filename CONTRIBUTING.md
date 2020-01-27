# Contribution Guidelines

## Auto-Formatting
ECL uses clang-format to auto-format the code. Currently it is using the clang-format-6.0.
The enforced style is based on the google style. Google's [Style Guide](https://google.github.io/styleguide/cppguide.html) is the place to look for advice.
The format is not enforced on all files. The list of files on which the auto-format checks are run on can be found in `tools/format.sh`

On Ubuntu (tested on 18.04) the following command can be used to check if the code is complying with the format requirements
```
make check_format
```
To auto-format the code run
```
make format
```

## Continuous Integration
There are multiple checks run on a submitted PR:

| Test  | Description |
| ------------- | ------------- |
| - **Build tests**         | Checks if the submitted code is building on various platforms. |
| - **Unit tests**          | Run checks if the code is satisfying test cases in `tests/` and report code coverage. |
| - **Format checks**       | Check if the files specified in `/tools/format.sh` match the style specified in `.clang-format`. Run [auto-formatting](#Auto-Formatting) |
| - **Firmware build tests**| Pulls PX4/Firmware and checks if the current branch of ECL compiles with it. It tries to checkout a branch in PX4/Firmware with the name matching the current ECL branch name. If a branch with that name does not exist then it will checkout master. |

## Unit tests
# How to run the tests
The test can be executed by running:
```
make test
```

# How to add a test
tbd

