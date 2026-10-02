## Checking numbers with code

You can run Python to check a numerical claim before you make it:

```
px4-sandbox-python -c '<python program>'
```

It runs Python 3 with numpy, sympy and pyulog in an isolated container: no
network, no credentials, the PR checkout read-only at `/work`, 90 seconds.
Pass the whole program after `-c` in single quotes, as one plain command;
nothing else on the line is allowed.

Use it when a finding rests on numbers you derived: matrix pseudo-inverses
and mixer outputs, filter and controller responses, unit and frame
conversions, overflow bounds, timing budgets. Rebuild the relevant math
from the code you read, run it, and quote the result in `body`. A number
you computed carries more weight than one you estimated, so say which it
is. Do not try to build or run PX4 itself.
