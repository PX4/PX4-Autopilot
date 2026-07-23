# Security Policy

## Supported Versions

The following versions receive security updates:

| Version | Supported          |
| ------- | ------------------ |
| 1.17.x  | :white_check_mark: |
| 1.16.x  | :white_check_mark: |
| < 1.16  | :x:                |

## Reporting a Vulnerability

We receive security vulnerability reports through GitHub Security Advisories.
If AI assistance contributed to your finding, do not file a private advisory:
read [AI-Assisted Discovery](#ai-assisted-discovery) instead.

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

## AI-Assisted Discovery

We align with the [Linux kernel security policy](https://docs.kernel.org/process/security-bugs.html)
on AI-assisted bug discovery.

If you used AI assistance to identify a bug, treat it as public. You may have
good reasons to believe it is not, but both the kernel security team and PX4
maintainers have seen bugs found this way surface across multiple researchers
at the same time, often on the same day. Instead of filing a private advisory,
open a pull request with a fix. If you cannot submit a fix yourself, first
search the [open pull requests](https://github.com/PX4/PX4-Autopilot/pulls)
for one that already addresses it; only if nothing exists, open an
[issue](https://github.com/PX4/PX4-Autopilot/issues/new/choose).

Your pull request makes the bug public, and that is expected. The reproducer
is different: never post it publicly, it works against vehicles in the field
until the fix ships. Describe the demonstrated impact in the pull request,
state that a reproducer is available, and maintainers will ask for it
privately if they need it.

### Responsible Use of AI to Find Bugs

A growing share of incoming security reports comes from AI-assisted code review.
It can surface real bugs in rarely exercised code, but it also floods
maintainers with low-quality reports. If AI tooling contributed to your finding,
the guidance above applies, and so do the following points:

- **Length**: Lead with a short summary: the affected module or file, the
  affected versions, and the demonstrated impact. AI tools tend to produce pages
  of sections and filler. Configure yours to write a concise, human-style
  report. Triage should never require scanning pages of text to find the
  essential facts.
- **Signal over noise**: Skip boilerplate sections and decorative structure that
  bury the facts. State only what you have demonstrated, in concrete PX4 terms:
  what an attacker on the MAVLink link, RC link, or CAN bus can actually do,
  such as write a parameter, change flight mode, arm the vehicle, or upload
  firmware. Do not enumerate speculative consequences.
- **Reproducer**: Have your tool produce a reproducer and test it yourself. A
  reproducer that works against SITL (`make px4_sitl`) is the gold standard. If
  it does not work, or the tool cannot produce one, treat the report itself as
  suspect. As stated above, do not post the reproducer publicly. Mention that
  it exists and share it with maintainers on request.
- **Propose a fix**: AI tools are often better at writing code than judging it.
  Have your tool propose a fix and test it before you report: in SITL at
  minimum, on hardware if the code path requires it. If the fix cannot be tested
  because it depends on hardware nobody has, the issue is likely not a real
  security bug. Fixes must follow the [contribution guide](CONTRIBUTING.md).

Reports that disregard these points risk being ignored.

Use common sense before filing. If the affected code is a driver for a
discontinued board or a rarely used peripheral and has not been touched in
years, exposed users are probably close to zero and the report is not worth a
maintainer's time. If the issue is trivial and publicly discoverable, take it
straight through the flow above: a public pull request, or an issue as a last
resort.

## Secure Development Practices

The PX4 development team applies the following practices to reduce security risk:

- **Code review**: All changes require peer review before merging.
- **Static analysis**: [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) runs on every pull request with warnings treated as errors.
- **Fuzzing**: A daily fuzzing pipeline using [Google fuzztest](https://github.com/google/fuzztest) tests MAVLink message handling and GNSS driver protocol parsing.
- **Input validation**: All external inputs (MAVLink messages, RC signals, sensor data) are validated against expected ranges before use.
- **Compiler hardening**: Builds use `-Wall -Werror`, stack protectors, and other hardening flags where supported by the target platform.
