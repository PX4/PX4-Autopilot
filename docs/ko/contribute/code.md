# 소스 코드 관리

## 분기 모델

PX4 프로젝트는 3가지 Git 분기 모델을 사용합니다.

- [main](https://github.com/PX4/PX4-Autopilot/tree/main) is by default unstable and sees rapid development.
- [beta](https://github.com/PX4/PX4-Autopilot/tree/beta) has been thoroughly tested. 비행 테스터를 위한 것입니다.
- [stable](https://github.com/PX4/PX4-Autopilot/tree/stable) points to the last release.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://docs.github.com/en/get-started/quickstart/github-flow).
그러나, 전세계의 역동적인 개발팀과 수시로 병합 작업을 진행합니다.

To contribute new functionality, [sign up for Github](https://docs.github.com/en/get-started/signing-up-for-github/signing-up-for-a-new-github-account), then [fork](https://docs.github.com/en/get-started/quickstart/fork-a-repo) the repository, [create a new branch](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository), add your [changes as commits](#commits-and-commit-messages), and finally [send a pull request](#pull-requests).
Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All code contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause) and all code must not impose any further constraints on the use.

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

- Maximum line length is 120 characters.

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

## 소스 내 문서 작업

PX4 개발자는 소스 내에서 적절한 문서를 작성하는 것이 좋습니다.

::: info

Source-code documentation standards are not enforced, and the code is currently inconsistently documented.
이보다 더 나아지길 바랍니다!

:::

현재 두 가지 소스 기반 문서 유형이 있습니다.

- `PRINT_MODULE_*` methods are used for both module run time usage instructions and for the [Modules & Commands Reference](../modules/modules_main.md) in this guide.
  - The API is documented [in the source code here](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381).
  - Good examples of usage include the [Application/Module Template](../modules/module_template.md) and the files linked from the modules reference.
- We encourage other in-source documentation _where it adds value/is not redundant_.

  :::tip

  Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.


:::

  - Do not add documentation that can trivially be inferred from C++ entity names.
  - ALWAYS specify units of variables, constants, and input/return parameters where they are defined.
  - 일반적으로 특이 사항이나 오류 처리에 대한 정보를 추가할 수 있습니다.
  - [Doxgyen](http://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`.
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

## 커밋과 커밋 메시지

Use descriptive, multi-paragraph commit messages for all non-trivial changes.
쉽게 이해할 수 있는 한 줄 요약과 자세한 세부정보도 기록하십시오.

```plain
Component: Explain the change in one sentence. Fixes #1234

Prepend the software component to the start of the summary
line, either by the module name or a description of it.
(e.g. "mc_att_ctrl" or "multicopter attitude controller").

If the issue number is appended as <Fixes #1234>, Github
will automatically close the issue when the commit is
merged to the master branch.

The body of the message can contain several paragraphs.
Describe in detail what you changed. Link issues and flight
logs either related to this fix or to the testing results
of this commit.

Describe the change and why you changed it, avoid to
paraphrase the code change (Good: "Adds an additional
safety check for vehicles with low quality GPS reception".
Bad: "Add gps_reception_check() function").

Reported-by: Name <email@px4.io>
```

**Use **`git commit -s`** to sign off on all of your commits.** This will add `signed-off-by:` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface-for-dirk/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.

## Pull Requests

Github [Pull Requests (PRs)](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) are the primary mechanism used to submit new functionality and bug fixes to PX4.

They include the new set of [commits](#commits-and-commit-messages) in your branch (relative the main branch), and a description of the changes.

The description should include:

- An overview of what the changes deliver; enough to understand the broad purpose of the code
- Links to related issues or supporting information.
- Information about what testing of the PR functionality has been done, with links to flight logs.
- Where possible, the results from general [Test Flights](../test_and_ci/test_flights.md) both before and after the change.
