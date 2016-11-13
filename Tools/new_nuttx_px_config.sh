#! /bin/bash

function fatal
{
	echo "$0: ERROR: $*"
	exit 1
}

# Find out what the base directory is.
BASEDIR="$(dirname $(dirname $(readlink -en "$0")))"
echo "BASEDIR=\"$BASEDIR\""
striplen=$((${#BASEDIR} + 2))

# BASEDIR may not contain a space, that's just too hard to get to work.
expr index "$BASEDIR" " " >/dev/null && fatal "it is not supported that BASEDIR contains a space."

# Make sure that worked.
test -f $BASEDIR/cmake/posix/px4_impl_posix.cmake || fatal "Failed to determine BASEDIR: '\$BASEDIR/cmake/posix/px4_impl_posix.cmake' is not a regular file."

# Did we get two arguments?
if [[ $# -lt 2 ]]; then
	echo "Usage:"
	echo "  $0 PROTOTYPE NEWNAME"
	echo
	echo "For example: $0 px4fmu-v4 awesome_new_board-v2"
	exit 1
fi

# Parse command line parameters.
oldname=
newname=
while [[ $# -gt 0 ]]
do
	case $1 in
		-*)
			fatal "Unknown option $1"
			;;
		*)
			if [ -z "$oldname" ]; then
				oldname="$1"
			elif [ -z "$newname" ]; then
				newname="$1"
			else
				fatal "Too many arguments."
			fi
			;;
	esac
	shift
done

echo "PROTOTYPE = \"$oldname\""

# Does that look like an existing old name?
if [ ! -f $BASEDIR/Images/$oldname.prototype ]; then
	fatal "\"$oldname\" doesn't look like an existing board name (there is no $BASEDIR/Images/$oldname.prototype)"
fi
if [ ! -d $BASEDIR/src/drivers/boards/$oldname ]; then
	fatal "\"$oldname\" doesn't look like an existing board name (no such directory $BASEDIR/src/drivers/boards/$oldname)"
fi
if [ ! -f $BASEDIR/cmake/configs/nuttx_"$oldname"_default.cmake -a ! -f $BASEDIR/cmake/configs/nuttx_"$oldname"_bootloader.cmake ]; then
	fatal "\"$oldname\" doesn't look like an existing board name (there is neither a $BASEDIR/cmake/configs/nuttx_"$oldname"_default.cmake nor a $BASEDIR/cmake/configs/nuttx_"$oldname"_bootloader.cmake)"
fi
if [ ! -d $BASEDIR/nuttx-configs/$oldname ]; then
	fatal "\"$oldname\" doesn't look like an existing board name (no such directory $BASEDIR/nuttx-configs/$oldname)"
fi

# Does the new name look like a new name?
if [[ $newname =~ ^[[:alnum:]_-]+$ ]]; then
	echo "NEWNAME = \"$newname\""
else
	fatal "NEWNAME may only contain alpha numeric characters, a dash or an underscore."
fi

# Change directory to BASEDIR.
cd "$BASEDIR" || fatal "Could not change directory to $BASEDIR"

# Make sure we don't accidently overwrite stuff.
if [ -f Images/$newname.prototype -o \
     -f cmake/configs/nuttx_"$newname"_default.cmake -o \
     -f cmake/configs/nuttx_"$newname"_bootloader.cmake -o \
     -d src/drivers/boards/$newname -o \
     -d nuttx-configs/$newname ]; then
  echo "\"$newname\" already exists! Please first delete it with the following command (in $BASEDIR):"
  echo "rm -rf Images/$newname.prototype cmake/configs/nuttx_"$newname"_default.cmake cmake/configs/nuttx_"$newname"_bootloader.cmake src/drivers/boards/$newname nuttx-configs/$newname"
  exit 1
fi

# Capitalize names.
oldboard=$(echo "$oldname" | tr '[:lower:]-' '[:upper:]_')
newboard=$(echo "$newname" | tr '[:lower:]-' '[:upper:]_')

# Do we have a default and/or bootloader?
[ ! -e cmake/configs/nuttx_"$oldname"_default.cmake ]; have_default=$?
[ ! -e cmake/configs/nuttx_"$oldname"_bootloader.cmake ]; have_bootloader=$?

function insertAfter # file line newText
{
   local file="$1" line="$2" newText="$3"
   sed -i -e "/$line/a"$'\\\n'"$newText" "$file"
}

for k in default bootloader; do
	eval needit=\$have_$k;
	if [[ $needit -eq 1 ]]; then
		# Copy cmake/configs/file(s).
		cp "cmake/configs/nuttx_${oldname}_${k}.cmake" "cmake/configs/nuttx_${newname}_${k}.cmake"
		git add "cmake/configs/nuttx_${newname}_${k}.cmake"
		sed -i -e "s%drivers/boards/${oldname}%drivers/boards/${newname}%" "cmake/configs/nuttx_${newname}_${k}.cmake"
		# If Makefile does not already have it, add 'check_$newname_[default|bootloader]'
		# to targets checks_[default|bootloader]s respectivily.
		if ! grep '^'$'\t'"check_${newname}_${k}"' \\$' Makefile >/dev/null; then
			insertAfter Makefile "^checks_${k}s:" $'\t'"check_${newname}_${k}"' \\'
		fi
	fi
done

# Copy remaining files.
cp Images/${oldname}.prototype Images/${newname}.prototype
git add Images/${newname}.prototype
cp -r nuttx-configs/${oldname} nuttx-configs/${newname}
git add nuttx-configs/${newname}
cp -r src/drivers/boards/${oldname} src/drivers/boards/${newname}
git add src/drivers/boards/${newname}
if [[ $have_bootloader -eq 1 ]]; then
	cp cmake/configs/uavcan_board_ident/${oldname}.cmake cmake/configs/uavcan_board_ident/${newname}.cmake
	git add cmake/configs/uavcan_board_ident/${newname}.cmake
	sed -i -r -e 's%\\"([^\\]*)\\"%\\"FIXME (was: \1)\\"%' cmake/configs/uavcan_board_ident/${newname}.cmake
fi

# Rename certain files.

TYPE_RE='(buttons|can|timer_config|i2c|init|led|pwr|sdio|spi|usb)'
oldstem=$(find "src/drivers/boards/$oldname" -type f -name '*.c' -o -name '*.cpp' | grep -E "_$TYPE_RE.[cp]*$" | sed -r -e 's%.*/%%;s%_'"$TYPE_RE"'\.[cp]*$%%' | sort -u | head -n 1)
newstem=$(echo "$newname" | sed -e 's/-v[0-9]*$//' | tr '[:upper:]-' '[:lower:]_')

echo "oldstem=\"$oldstem\""
echo "newstem=\"$newstem\""

FILES=$(find src/drivers/boards/${newname} -regextype egrep -regex ".*/$oldstem[0-9]*_$TYPE_RE\.[cp]*")
for f in $FILES; do
	nf=$(echo $f | sed -r -e 's%^(.*/)'"$oldstem"'([0-9]*_'"$TYPE_RE"'\.[cp]*)$%\1'"$newstem"'\2%')
	if [[ "$f" != "$nf" ]]; then
		git mv "$f" "$nf"
	fi
	git add "$nf"
	bf=$(basename "$f")
	bnf=$(basename "$nf")
	sed -i -e "s%$bf%$bnf%" "$nf"
done

oldconfig=CONFIG_ARCH_BOARD_$oldboard
newconfig=CONFIG_ARCH_BOARD_$newboard

# Fixup copied files.
sed -i -r -e 's%(MODULE[[:space:]]+.*)'"$oldname"'%\1'"$newname"'%;s%^([[:space:]]+)'"$oldstem"'([0-9]*_'"$TYPE_RE"'\.)%\1'"$newstem"'\2%' "src/drivers/boards/${newname}/CMakeLists.txt"
sed -i -r -e 's%"'"$oldboard"'"%"'"$newboard"'"%' "src/drivers/boards/${newname}/board_config.h"
sed -i -r -e 's%'"$oldname"'%'"$newname"'%' "nuttx-configs/${newname}/nsh/Make.defs"
sed -i -r -e 's%'"$oldname"'%'"$newname"'%;s%'"$oldconfig"'%'"$newconfig"'%;s%(CONFIG_CDCACM_PRODUCTSTR=)"([^"]*)"%\1"FIXME (was: \2)"%' "nuttx-configs/${newname}/nsh/defconfig"

# If src/modules/gpio_led/gpio_led.c does not already contain it,
# add defined($newconfig) where defined($oldconfig) exists.
if ! grep 'defined('"${newconfig}"')' 'src/modules/gpio_led/gpio_led.c' >/dev/null; then
	insertAfter src/modules/gpio_led/gpio_led.c "defined(${oldconfig})"' || \\$' $'\t'"defined(${newconfig})"' || \\'
	sed -i -e 's/\(defined('"${oldconfig}"')$\)/\1 || \\'$'\\n\\t'"defined(${newconfig})/" 'src/modules/gpio_led/gpio_led.c'
fi

# Make some changes to Images/$(newname).prototype
sed -i -r -e 's%("(magic|description|summary)": ")([^"]*)(",).*%\1FIXME (was: \3)\4%' "Images/${newname}.prototype"

echo "*** The following files contain a reference to $oldconfig (this might take a while):"
find . -path './build_*' -o -path './.git' -o -name 'defconfig' -prune -o -type f -exec grep -l -- "$oldconfig" {} \;
echo "*** Run 'git diff' to check the changes that this script already made relative to the copied prototype files."
echo "*** Use 'git status' to see other (added) files, that likely need fixing."
