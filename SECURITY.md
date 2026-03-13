# Security Policy

## Supported Versions

The following versions receive security updates:

| Version | Supported          |
| ------- | ------------------ |
| 1.16.x  | :white_check_mark: |
| < 1.16  | :x:                |

## Reporting a Vulnerability

We receive security vulnerability reports through GitHub Security Advisories.

To begin a report, go to the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository
and click on the **Security** tab. If you are on mobile, click the **...** dropdown menu, then click **Security**.

Click **Report a Vulnerability** to open the advisory form. Fill in the advisory details form.
Make sure your title is descriptive and the description contains all relevant details needed
to verify the issue. We welcome logs, screenshots, photos, and videos.

At the bottom of the form, click **Submit report**.

## Response Process

1. **Acknowledgment**: The maintainer team will acknowledge your report within **7 days**.
2. **Triage**: We will assess severity and impact and communicate next steps.
3. **Disclosure**: We coordinate disclosure with the reporter. We follow responsible disclosure practices and will credit reporters in the advisory unless they request anonymity.

If you do not receive acknowledgment within 7 days, please follow up by emailing the [release managers](MAINTAINERS.md).

## Secure Development Practices

The PX4 development team applies the following practices to reduce security risk:

- **Code review**: All changes require peer review before merging.
- **Static analysis**: [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) runs on every pull request with warnings treated as errors.
- **Fuzzing**: A daily fuzzing pipeline using [Google fuzztest](https://github.com/google/fuzztest) tests MAVLink message handling and GNSS driver protocol parsing.
- **Input validation**: All external inputs (MAVLink messages, RC signals, sensor data) are validated against expected ranges before use.
- **Compiler hardening**: Builds use `-Wall -Werror`, stack protectors, and other hardening flags where supported by the target platform.
