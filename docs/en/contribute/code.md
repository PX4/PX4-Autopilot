# Source Code Management

## Branching Model

The PX4 project uses a three-branch Git branching model:

- [main](https://github.com/PX4/PX4-Autopilot/tree/main) is by default unstable and sees rapid development.
- [beta](https://github.com/PX4/PX4-Autopilot/tree/beta) has been thoroughly tested. It's intended for flight testers.
- [stable](https://github.com/PX4/PX4-Autopilot/tree/stable) points to the last release.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://docs.github.com/en/get-started/using-github/github-flow).
However, due to the global team and fast moving development we might resort to merges at times.

To contribute new functionality, [sign up for Github](https://docs.github.com/en/get-started/using-github/github-flow), then [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) the repository, [create a new branch](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository), add your [changes as commits](#commits-and-commit-messages), and finally [send a pull request](#pull-requests).
Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All code contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/license/BSD-3-Clause) and all code must not impose any further constraints on the use.

## Code Style

PX4 uses the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html), with the following (minimal) modifications:

::: info

Not all PX4 source code matches the style guide, but any _new code_ that you write should do so — in both new and existing files.
If you update an existing file you are not required to make the whole file comply with the style guide, just the code you've modified.

:::

### Tabs

- Tabs are used for indentation (equivalent to 8 spaces).
- Spaces are used for alignment.

### Line Length

- Maximum line length is 140 characters.

### File Extensions

- Source files use extension `*.cpp` instead of `*.cc`.

### Function and Method Names

- `lowerCamelCase()` is used for functions and methods to _visually_ distinguish them from `ClassConstructors()` and `ClassNames`.

### Private Class Member Variable Names

- `_underscore_prefixed_snake_case` is used for private class member variable names, as oppose to `underscore_postfixed_`.

### Class Privacy Keywords

- _zero_ spaces before `public:`, `private:`, or `protected:` keywords.

### Example Code Snippet

```cpp
class MyClass {
public:

        /**
         * @brief Description of what this function does.
         *
         * @param[in] input_param Clear description of the input [units]
         * @return Whatever we are returning [units]
         */
        float doSomething(const float input_param) const {
                const float in_scope_variable = input_param + kConstantFloat;
                return in_scope_variable * _private_member_variable;
        }

        void setPrivateMember(const float private_member_variable) { _private_member_variable = private_member_variable; }

        /**
         * @return Whatever we are "getting" [units]
         */
        float getPrivateMember() const { return _private_member_variable; }

private:

        // Clear description of the constant if not completely obvious from the name [units]
        static constexpr float kConstantFloat = ...;

        // Clear description of the variable if not completely obvious from the name [units]
        float _private_member_variable{...};
};
```

## In-Source Documentation

PX4 developers are encouraged to create appropriate in-source documentation.

::: info

Source-code documentation standards are not enforced, and the code is currently inconsistently documented.
We'd like to do better!

:::

Currently we have two types of source-based documentation:

- `PRINT_MODULE_*` methods are used for both module run time usage instructions and for the [Modules & Commands Reference](../modules/modules_main.md) in this guide.
  - The API is documented [in the source code here](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381).
  - Good examples of usage include the [Application/Module Template](../modules/module_template.md) and the files linked from the modules reference.
- We encourage other in-source documentation _where it adds value/is not redundant_.

  :::tip

  Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.

  :::
  - Do not add documentation that can trivially be inferred from C++ entity names.
  - ALWAYS specify units of variables, constants, and input/return parameters where they are defined.
  - Commonly you may want to add information about corner cases and error handling.
  - [Doxgyen](https://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`.
    A good example of usage is [src/modules/events/send_event.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/events/send_event.h).

Please avoid "magic numbers", for example, where does this number in the conditional come from? What about the multiplier on yaw stick input?

```cpp
if (fabsf(yaw_stick_normalized_input) < 0.1f) {
        yaw_rate_setpoint = 0.0f;
}
else {
        yaw_rate_setpoint = 0.52f * yaw_stick_normalized_input;
}
```

Instead, define the numbers as named constants with appropriate context in the header:

```cpp
// Deadzone threshold for normalized yaw stick input
static constexpr float kYawStickDeadzone = 0.1f;

// [rad/s] Deadzone threshold for normalized yaw stick input
static constexpr float kMaxYawRate = math::radians(30.0f);
```

and update the source implementation.

```cpp
if (fabsf(yaw_stick_normalized_input) < kYawStickDeadzone) {
        yaw_rate_setpoint = 0.0f;
}
else {
        yaw_rate_setpoint = kMaxYawRate * yaw_stick_normalized_input;
}
```

## Commits and Commit Messages

PX4 uses [conventional commits](https://www.conventionalcommits.org/) for all commit messages and PR titles.

### Format

```
type(scope): short description of the change
```

Where **type** is the category of change (`feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`, `style`, `chore`, `revert`) and **scope** is the module or area affected (e.g. `ekf2`, `mavlink`, `navigator`). See the full [types and scopes tables](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) in CONTRIBUTING.md.

Append `!` before the colon to mark a breaking change: `feat(ekf2)!: remove deprecated API`.

### Examples

```
feat(ekf2): add height fusion timeout. Fixes #1234

The previous implementation did not handle the case where
height fusion data stops arriving mid-flight. This adds a
configurable timeout that falls back to barometric height.

Tested in SITL with simulated sensor dropout.

Signed-off-by: Your Name <your@email.com>
```

The body of the message can contain several paragraphs. Describe in detail what you changed and why. Link related issues and flight logs. Describe the change and why you made it, rather than paraphrasing the code change.

**Use `git commit -s` to sign off on all of your commits.** This adds `Signed-off-by:` with your name and email as the last line.

## Pull Requests

Github [Pull Requests (PRs)](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) are the primary mechanism used to submit new functionality and bug fixes to PX4.

They include the new set of [commits](#commits-and-commit-messages) in your branch (relative the main branch), and a description of the changes.

The description should include:

- An overview of what the changes deliver; enough to understand the broad purpose of the code
- Links to related issues or supporting information.
- Information about what testing of the PR functionality has been done, with links to flight logs.
- Where possible, the results from general [Test Flights](../test_and_ci/test_flights.md) both before and after the change.
