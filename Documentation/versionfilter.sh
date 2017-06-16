#!/bin/sh
git log --pretty=format:"Last change: commit %h - %aN, %ar : %s" -1 $1 || echo no git