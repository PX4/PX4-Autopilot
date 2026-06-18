#!/usr/bin/env python3
import sys
import argparse
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator as EA

parser = argparse.ArgumentParser()
parser.add_argument('file', type=str)
parser.add_argument('tag', type=str, default=None, nargs="?")

args = parser.parse_args()

ea=EA(args.file)
print(f"Loading: {args.file}", file=sys.stderr)
ea.Reload()
if args.tag is None:
    for tag in ea.Tags()["scalars"]:
        events = ea.Scalars(tag)
        print(f"{tag}: {len(events)}", file=sys.stderr)
else:
    [print(f'{e.step},{e.value}') for e in ea.Scalars(args.tag)]
