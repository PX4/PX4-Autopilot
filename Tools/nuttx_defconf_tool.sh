#! /bin/bash

if [ $# -eq 0 ]
  then
    echo "Usage: $0 <path to defconfig>"
    exit 1
fi
configsdir=nuttx-configs
defconf=$1
configsrc=$configsdir${defconf##*$configsdir}

if [ ! -f  $defconf ]
  then
    echo "$defconf does not exist"
    exit 2
fi


lastconf=$defconf.last
chunk=$defconf.chunk

echo $configsrc
git show HEAD:$configsrc > $lastconf

lead='^# Board Selection$'
tail='^# Common Board Options$'

cat $lastconf | sed -n "/$lead/,/$tail/p" | sed '1d;$d' > $chunk

echo "Fix up Board Selection"
sed -i -e "/$lead/,/$tail/{ /$lead/{p; r $chunk
        }; /$tail/p; d }" $defconf


if grep --quiet CONFIG_START_YEAR $lastconf ; then
  lead='^CONFIG_START_YEAR='
  tail='^CONFIG_START_DAY='
  cat $lastconf | sed -n "/$lead/,/$tail/p" > $chunk
  lead='^# Clocks and Timers$'
  echo "Fix up Clocks and Timers"
  sed -i -e "/$lead/{N;{r $chunk
    }}" $defconf
else
 echo not found
fi

rm $lastconf
rm $chunk