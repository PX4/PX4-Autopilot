
# PX4-Baseline - Upstream Tracking Policy

Upstream Repository:
- https://github.com/PX4/PX4-Autopilot

Tracking Strategy:
- Track official PX4 release tags only (e.g. 'release/X.XX')
- No floating branches
- No custom commits

Update Process:
- Fetch upstream
- Checkout target release tag
- Fast-forward merge into this fork
- Tag internally with matching version

