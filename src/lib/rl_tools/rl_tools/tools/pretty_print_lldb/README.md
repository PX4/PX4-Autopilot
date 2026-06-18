To debug this, start some target that includes the desired datastructures (e.g. a small test containing a lot of tensors) in debug mode and run:

```
command script import tools/pretty_print_lldb/debug.py
```

in lldb


this dumps all the types in the `rl_tools` namespace into `lldb_type_dump.txt`. There you can desing the regexes to match and extract features from the types