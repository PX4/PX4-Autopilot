#!/bin/bash
#
# Poor man's sampling profiler for NuttX.
#
# The stack folding script was inspired by stackcollapse-gdb.pl from the FlameGraph project.
#
# Usage: Install flamegraph.pl in your PATH, define the variables below, configure your .gdbinit, run the script and go
#        have a coffee. When you're back, you'll see the flamegraph. Note that frequent calls to GDB significantly
#        interfere with normal operation of the target, which means that you can't profile real-time tasks with it.
#
# Requirements: ARM GDB with Python support
#

root=$(dirname $0)/..

nsamples=100
sleeptime=0.01    # Doctors recommend 7-8 hours a day
taskname=uavcan
exe=$root/Build/px4fmu-v2_default.build/firmware.elf

set -e

stacksfile=/tmp/$taskname-stacks.log

cd $root
rm -f $stacksfile
echo "Sampling..."

for x in $(seq 1 $nsamples)
do
    arm-none-eabi-gdb $exe --batch -ex "set print asm-demangle on" \
                                   -ex "source $root/Debug/Nuttx.py" \
                                   -ex "show mybt $taskname" \
        2> /dev/null \
        | sed -n 's/0\.0:\(#.*\)/\1/p' \
        >> $stacksfile
    echo -e '\n\n' >> $stacksfile
    echo -ne "\r$x/$nsamples"
    sleep $sleeptime
done

echo
echo "Stacks saved to $stacksfile"

cat $stacksfile | perl -e 'my $current = "";
my %stacks;
while(<>) {
  if(m/^#[0-9]*\s*0x[a-zA-Z0-9]*\s*in (.*) at/) {
    if ($1 ne "None") {
      if ($current eq "") { $current = $1; }
      else { $current = $1 . ";" . $current; }
    }
  } elsif(!($current eq "")) {
    $stacks{$current} += 1;
    $current = "";
  }
}
foreach my $k (sort { $a cmp $b } keys %stacks) {
  print "$k $stacks{$k}\n";
}' > /tmp/$taskname-folded.txt

echo "Folded stacks saved to /tmp/$taskname-folded.txt"

cat /tmp/$taskname-folded.txt | flamegraph.pl --fontsize=8 --width=1800 > /tmp/$taskname-flamegraph.svg

xdg-open /tmp/$taskname-flamegraph.svg
