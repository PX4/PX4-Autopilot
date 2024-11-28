#!/usr/bin/env/python
import json
content =open('/tmp/tracer.json', 'r').read()
for entry in content.split('+++++++'):
  if not entry.strip(): continue
  e = json.loads(entry)
  print(e['op'], e['args'])
  print(e['bt'])

