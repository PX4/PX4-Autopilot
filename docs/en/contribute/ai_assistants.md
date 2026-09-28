# AI Coding Assistants

AI-assisted contributions are welcome in PX4.
AI coding assistants are development tools, like compilers and static analyzers, and contributions made with their help are held to exactly the same standard as any other contribution: the [coding standards](../contribute/code.md), [commit conventions](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md), testing requirements, and review process apply unchanged.

This page defines the additional rules that apply when AI tools are used.
They exist to keep authorship, accountability, and licensing unambiguous, and to keep review manageable for maintainers.

::: info
Using AI tools is optional.
Disclosing their use is not.
:::

## Authorship and Accountability

The human submitter is the author of the contribution.

- You must understand every line you submit, be able to explain it, and be prepared to defend it in review.
- Responsibility for correctness, safety, licensing, and testing never transfers to a tool.
  PX4 is safety-critical software that flies real vehicles; "the AI wrote it" is never an acceptable answer in review.
- An AI tool is never an author or co-author.
  Do not add `Co-Authored-By` tags naming an AI tool.

## Signed-off-by and the Developer Certificate of Origin

A `Signed-off-by` tag is the human author's certification under the [Developer Certificate of Origin](https://developercertificate.org/).

- An AI tool is never the author of a contribution and must never appear in a `Signed-off-by` tag.
- Tooling acting on your instruction, including an AI agent, may apply your sign-off mechanically, the same way `git commit -s` does.
  The certification remains yours.
- By signing off you certify that you have personally reviewed the contribution and have the right to submit it under the project license.
  Never allow your sign-off to be applied to changes you have not reviewed.

## Licensing

All code contributions must be compatible with the [BSD 3-clause license](../contribute/licenses.md) and must not impose any further constraints.

When using AI tools you are additionally responsible for the conditions of the [Linux Foundation Generative AI Policy](https://www.linuxfoundation.org/legal/generative-ai):

- The terms of service of the AI tool you use must not conflict with the project license.
- If the tool's output includes pre-existing copyrighted material from a third party, you must have the right to submit it, and must include the required notice, attribution, and license information.

## Disclosure (Required)

Every commit that contains AI-generated or AI-assisted content (code, documentation, or commit message text) must carry a disclosure trailer in the commit message body:

```
Assisted-by: NAME:MODEL
```

`NAME` identifies the tool and `MODEL` the specific model used, for example:

```
Assisted-by: Claude:claude-fable-5
Assisted-by: Copilot:gpt-5
```

- Conventional development tooling does not need to be disclosed: compilers, formatters (`make format`), linters, git, and classic editor autocomplete are not AI assistance in this sense.
- Like the sign-off, disclosure operates on good faith: reviewers cannot reliably detect AI usage and will not try to.
  Undisclosed AI use discovered after the fact is treated as a misrepresentation and is grounds for reverting the contribution.

## Expectations

These rules keep review sustainable.
Contributions that ignore them may be closed by maintainers without detailed review.

- **Test claims must be real.**
  The [testing requirements](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#test-your-changes) apply unchanged: state exactly what you ran (SITL, bench test, flight test) and provide logs where they apply.
  Never let a tool describe testing that did not happen.
  An AI cannot have flown your vehicle.
- **No unsolicited bulk changes.**
  Do not submit large AI-generated refactors, style sweeps, or "cleanup" pull requests that nobody asked for.
  Discuss the idea in an issue or on the [dev call](../contribute/dev_call.md) first.
- **Issues and security reports must be human-verified.**
  Reproduce the problem yourself before filing.
  Unverified AI-generated findings cost maintainers real time and erode trust.
- **Review discussion is between humans.**
  Using an AI tool to help you understand a review comment is fine; posting model output you do not understand as your reply is not.
  AI-assisted review is acceptable when clearly labeled as such; presenting undisclosed AI review as your own reading is not.
