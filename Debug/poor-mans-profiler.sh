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

set -e
root=$(dirname $0)/..

function die()
{
    echo "$@"
    exit 1
}

function usage()
{
    echo "Invalid usage. Supported options:"
    cat $0 | sed -n 's/^\s*--\([^)\*]*\).*/\1/p' # Don't try this at home.
    exit 1
}

which flamegraph.pl > /dev/null || die "Install flamegraph.pl first"

#
# Parsing the arguments. Read this section for usage info.
#
nsamples=0
sleeptime=0.1    # Doctors recommend 7-8 hours a day
taskname=
elf=$root/Build/px4fmu-v2_default.build/firmware.elf
append=0
fgfontsize=10
fgwidth=1900

for i in "$@"
do
    case $i in
        --nsamples=*)
            nsamples="${i#*=}"
            ;;
        --sleeptime=*)
            sleeptime="${i#*=}"
            ;;
        --taskname=*)
            taskname="${i#*=}"
            ;;
        --elf=*)
            elf="${i#*=}"
            ;;
        --append)
            append=1
            ;;
        --fgfontsize=*)
            fgfontsize="${i#*=}"
            ;;
        --fgwidth=*)
            fgwidth="${i#*=}"
            ;;
        *)
            usage
            ;;
    esac
    shift
done

#
# Temporary files
#
stacksfile=/tmp/pmpn-stacks.log
foldfile=/tmp/pmpn-folded.txt
graphfile=/tmp/pmpn-flamegraph.svg
gdberrfile=/tmp/pmpn-gdberr.log

#
# Sampling if requested. Note that if $append is true, the stack file will not be rewritten.
#
cd $root

if [[ $nsamples > 0 && "$taskname" != "" ]]
then
    [[ $append = 0 ]] && (rm -f $stacksfile; echo "Old stacks removed")

    echo "Sampling the task '$taskname'..."

    for x in $(seq 1 $nsamples)
    do
        arm-none-eabi-gdb $elf --batch -ex "set print asm-demangle on" \
                                       -ex "source $root/Debug/Nuttx.py" \
                                       -ex "show mybt $taskname" \
            2> $gdberrfile \
            | sed -n 's/0\.0:\(#.*\)/\1/p' \
            >> $stacksfile
        echo -e '\n\n' >> $stacksfile
        echo -ne "\r$x/$nsamples"
        sleep $sleeptime
    done

    echo
    echo "Stacks saved to $stacksfile"
else
    echo "Sampling skipped - set 'nsamples' and 'taskname' to re-sample."
fi

#
# Folding the stacks.
#
[ -f $stacksfile ] || die "Where are the stack samples?"

cat $stacksfile | perl -e 'use File::Basename;
my $current = "";
my %stacks;
while(<>) {
  if(m/^#[0-9]*\s*0x[a-zA-Z0-9]*\s*in (.*) at (.*)/) {
    my $x = $1 eq "None" ? basename($2) : $1;
    if ($current eq "") { $current = $x; }
    else { $current = $x . ";" . $current; }
  } elsif(!($current eq "")) {
    $stacks{$current} += 1;
    $current = "";
  }
}
foreach my $k (sort { $a cmp $b } keys %stacks) {
  print "$k $stacks{$k}\n";
}' > $foldfile

echo "Folded stacks saved to $foldfile"

#
# Graphing.
#
cat $foldfile | flamegraph.pl --fontsize=$fgfontsize --width=$fgwidth > $graphfile
echo "FlameGraph saved to $graphfile"

# On KDE, xdg-open prefers Gwenview by default, which doesn't handle interactive SVGs, so we need a browser.
# The current implementation is hackish and stupid. Somebody, please do something about it.
opener=xdg-open
which firefox       > /dev/null && opener=firefox
which google-chrome > /dev/null && opener=google-chrome

$opener $graphfile
