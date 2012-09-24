#/bin/sh

git fetch upstream
# Fetches any new changes from the original repo

git merge upstream/master
# Merges any changes fetched into your working files

