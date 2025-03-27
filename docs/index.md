---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

hero:
  name: "PX4"
  text: "User Guide"
  tagline: PX4 is the Open Source Autopilot for Professional Drone Developers.
  image: /logo_pro_small.png
  actions:
    - theme: brand
      text: Read the docs
      link: /en/index.md
    - theme: alt
      text: Website
      link: https://px4.io/
    - theme: alt
      text: Source code on GitHub
      link: https://github.com/PX4/PX4-Autopilot

features:
  - title: Modular Architecture
    details: PX4 is highly modular and extensible both in terms of hardware and software. It utilizes a port-based architecture – which means when developers add components, the extended system does not lose robustness or performance. 
  - title: Open Source
    details: PX4 is co-developed with a global development community. The flightstack is not just fulfilling the needs of one lab or one company, but has been intended as a general toolkit and is widely used and adopted in the industry.
  - title: Configurability
    details: PX4 offers optimised APIs and SDKs for developers working with integrations. All the modules are self-contained and can be easily exchanged against a different module without modifying the core. Features are easy to deploy and reconfigure.
  - title: Autonomy Stack
    details: PX4 is designed to be deeply coupled with embedded computer vision for autonomous capabilities . The framework lowers the barrier of entry for developers working on localization and obstacle detection algorithms.
  - title: Real-world Validation
    details: Thousands of commercial PX4 based systems have been deployed worldwide. In parallel, dedicated flight test team clocking up thousands of flight hours each month running hardware and software tests to ensure the codebase’s safety and reliability.
  - title: Permissive License
    details: PX4 is free to use and modify under the terms of the permissive BSD 3-clause license. Which means the software also allows proprietary use and allows the releases under the license to be incorporated into proprietary products.
  - title: Interoperability
    details: Beyond being a robust flight stack, PX4 offers an ecosystem of supported devices. The project also leads the standardarization effort for the advancement of communications, peripherals integration, and power management solutions.

  - title: Powerful Safety Features
    details: Great safety features including automated failsafe behaviour, support for different return modes, parachutes etc. are by default already included in the codebase. The features are easily configurable and tunable for custom systems.
	
search: false
footer: BSD 3-clause license
---

<!-- <Redirect to="/en/README.md" /> -->