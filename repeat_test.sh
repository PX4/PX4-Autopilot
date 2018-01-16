#! /bin/bash

for i in {1..20}
do
   echo -e "\n-------------------\n----- push $i ------\n-------------------\n"
   git reset --soft HEAD~1
   git commit --no-gpg-sign -m "WIP"
   git push --force px4 pr-pwm_out_sim-debug
   echo -e "\nsleeping ..."
   sleep 10m
done
