#!/bin/bash

# create uorb graphs and move to docs
make uorb_graphs
cp Tools/uorb_graph/*.json docs/public/middleware/
# create failsafe sim
# uorb message docs
# parameter metadata
# airframe reference
# module docs
