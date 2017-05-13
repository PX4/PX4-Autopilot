#! /bin/bash

# Written by Carlo Wood, September/October 2016.

function fatal
{
	echo "$0: ERROR: $*"
	exit 1
}

# Make sure we're not having a broken gawk.
AWK_VERSION=$(awk -V | head -n 1)
if [[ $AWK_VERSION =~ ^GNU\ Awk\ 4\.[0-9]+\.[0-9]+ ]]; then
	AWK_VERSION=$(echo $AWK_VERSION | sed -e 's/GNU Awk \(4\.[0-9]*\.[0-9]*\).*/\1/')
	if [[ $AWK_VERSION =~ ^4\.0*([2-9]+|1\.0*[2-9]+) ]]; then
		fatal "Your version of awk ($AWK_VERSION) is broken. Please use version 4.1.1 or lower."
	fi
fi
echo "AWK_VERSION=$AWK_VERSION"

# Find out what the base directory is.
BASEDIR="$(dirname $(dirname $(readlink -en "$0")))"
echo "BASEDIR=\"$BASEDIR\""
striplen=$((${#BASEDIR} + 2))

# BASEDIR may not contain a space, that's just too hard to get to work.
expr index "$BASEDIR" " " >/dev/null && fatal "it is not supported that BASEDIR contains a space."

# Make sure that worked.
test -f $BASEDIR/cmake/posix/px4_impl_posix.cmake || fatal "Failed to determine BASEDIR: '\$BASEDIR/cmake/posix/px4_impl_posix.cmake' is not a regular file."

# Parse command line parameters.
debug=0	# Set to non-zero to enable debugging.
force=0	# Set to 1 to force running of script even when there are uncommitted changes.
merge=0	# Set to 1 when merging a branch that didn't run this script into master that did already run this script.
while [[ $# -gt 0 ]]
do
	case $1 in
		--debug)
			debug=1
			;;
		--force)
			force=1
			;;
		--merge)
			force=1
			merge=1
			fatal "--merge is not implemented yet."
			;;
		-*)
			fatal "Unknown option $1"
			;;
		--|*)
			break
			;;
	esac
	shift
done
non_option_arguments=$#
if [ $non_option_arguments -eq 0 -a $debug -ne 0 ]; then
	fatal "--debug screws up the source files with debug output! You must provide a single filename to run on."
fi

# Better not run this script with changes that still need to be committed.
cd "$BASEDIR" || fatal "Could not change directory to \"$BASEDIR\""
if ! git diff-index --quiet HEAD --; then
	if [ $non_option_arguments -ne 0 -o $force -eq 1 ]; then
		if [ $force -eq 1 ]; then
			echo "Uncommitted changes, but running anyway because --force is used."
		else
			echo -n "WARNING: You have uncommitted changes (use --force to remove this warning). Run anyway? [y/N] "
			read answer
			if [ "x$answer" != "xy" -a "x$answer" != "xY" ]; then exit 0; fi
		fi
	else
		fatal "Your working directory has uncommitted changes (see 'git status')! Bailing out."
	fi
fi

# Find a reasonable tmp directory.
# First make a list of all build directories by looking for a CMakeCache.txt in them. Sort them so the most recent one is first.
CMAKECACHE_FILES=$(find "$BASEDIR" -mindepth 2 -maxdepth 2 -type f -name CMakeCache.txt -wholename "$BASEDIR/build_*/CMakeCache.txt" | xargs /bin/ls -td)
# Make a list of all candidate tmp directories.
TMPDIRS=
for f in $CMAKECACHE_FILES; do
	if [ -d $(dirname $f)/tmp ]; then
		TMPDIRS+=" $(dirname $f)/tmp"
	fi
done
# Put BASEDIR first in case there are no build directories because /tmp is probably on a different file system.
TMPDIRS+=" $BASEDIR /tmp ."
# Pick the first one that is actually writable.
for tmp in $TMPDIRS; do
	TMPDIR="$tmp"
	if [ -w "$TMPDIR" ]; then
		break;
	fi
done
test -n "$TMPDIR" || fatal "Can not find a writable tmp directory."
echo "TMPDIR=\"$TMPDIR\""

# Make a list of all source and header files that we need to fix.
# List of directories that we don't want to touch.
EXCLUDE_FOLDERS=".git unittests Tools"
EXCLUDE_PATTERNS="examples matlab/scripts tests test unit_test *_test *_tests test_* apps/test_* UnitTests"
# A regular expression for the exclude patterns.
EXCLUDE_PATTERNS_RE="($(echo $EXCLUDE_PATTERNS | sed -e 's/\*/[^\/]*/g;s/ /|/g'))"
# Extensions of files that we do want to change (leaving out .y and .py for now).
C_EXTENSIONS=".c .c_in .c_shipped"
H_EXTENSIONS=".h .h.in .h_template"
CXX_EXTENSIONS=".cc .cpp .cpp.in .cxx .cpp_template"
HXX_EXTENSIONS=".hh .hpp .hxx"
# The regular expression that we consider to be an #include.
INCLUDE_RE='^[[:space:]]*#[[:space:]]*include[[:space:]]*[<"]'
# Regular expression for empty lines.
EMPTY_LINE_RE='^[[:space:]]*$'
# Regular expression for one-line comments.
COMMENT_LINE_RE='^[[:space:]]*(\/\/.*|\/\*([^*]|\*+[^\/*])*\*+\/[[:space:]]*)$'
# Regular expression for a #define (on one line).
DEFINE_RE='^[[:space:]]*#[[:space:]]*define[[:space:]].*[^\\]$'
# Regular expression for an #if[[n]def].
IF_RE='^[[:space:]]*#[[:space:]]*if(n?def)?[[:space:]]'
# Regular expression for an #endif.
ENDIF_RE='^[[:space:]]*#[[:space:]]*endif($|[^[:alnum:]])'
# Regular expression for header file extension.
HEADER_RE="($(echo $H_EXTENSIONS $HXX_EXTENSIONS | sed -e 's/\./\\./g;s/ /|/g'))"
# Regular expression for C++ source and header files.
CXXSRC_RE="($(echo $CXX_EXTENSIONS $HXX_EXTENSIONS | sed -e 's/\./\\./g;s/ /|/g'))"
# List of standard C header files. Note that cfcntl, cshed and cunistd are NOT standard header files, even though they are in NuttX/nuttx/include/cxx.
REAL_STDC_HEADERS_RE='(cassert|ccomplex|cctype|cerrno|cfenv|cfloat|cinttypes|ciso646|climits|clocale|cmath|csetjmp|csignal|cstdalign|cstdarg|cstdbool|cstddef|cstdint|cstdio|cstdlib|cstring|ctgmath|ctime|cuchar|cwchar|cwctype)'
STDC_HEADERS=$(find "$BASEDIR/NuttX/nuttx/include/cxx" -mindepth 1 -maxdepth 1 -type f | xargs basename -a | grep -E "$REAL_STDC_HEADERS_RE" | xargs echo)
# Regular expression of standard C header files, but with the leading 'c' stripped.
STDC_HEADERS_RE="($(echo $STDC_HEADERS | sed -e 's/^c//;s/ c/|/g'))"
# Actual list of standard C header files.
# List of standard C++ header files.
REAL_STDCXX_HEADERS_RE='(algorithm|any|array|atomic|bitset|cassert|ccomplex|cctype|cerrno|cfenv|cfloat|chrono|cinttypes|ciso646|climits|clocale|cmath|codecvt|complex|condition_variable|csetjmp|csignal|cstdalign|cstdarg|cstdbool|cstddef|cstdint|cstdio|cstdlib|cstring|ctgmath|ctime|cuchar|cwchar|cwctype|deque|exception|execution|filesystem|forward_list|fstream|functional|future|initializer_list|iomanip|ios|iosfwd|iostream|istream|iterator|limits|list|locale|map|memory|memory_resource|mutex|new|numeric|optional|ostream|queue|random|ratio|regex|scoped_allocator|set|shared_mutex|sstream|stack|stdexcept|streambuf|string|string_view|strstream|system_error|thread|tuple|typeindex|typeinfo|type_traits|unordered_map|unordered_set|utility|valarray|variant|vector)'
STDCXX_HEADERS=$(find "$BASEDIR/NuttX/misc/uClibc++/include/uClibc++" -mindepth 1 -maxdepth 1 -type f | xargs basename -a | grep -E "$REAL_STDCXX_HEADERS_RE" | grep -E -v "$REAL_STDC_HEADERS_RE" | xargs echo)
# Regular expression of C++ header files.
STDCXX_HEADERS_RE="($(echo $STDCXX_HEADERS | sed -e 's/ /|/g'))"
# Regular expression for #pragma once.
PRAGMA_ONCE_RE='^#pragma once'
# Regular expression to recognize the start of a C-comment block.
COMMENT_BEGIN_RE='(^|[^\/])\/\*([^*]|\*+($|[^\/*]))*$'
# Regular expression to recognize the end of a C-comment block.
COMMENT_END_RE='\*\/'
# Regular expression to match C++ unsafe headers. We currently don't have any C++ unsafe headers, do we?
# v2.0/standard/mavlink.h is not unsafe, but this way the script will leave it alone and not
# move it above function declarations that need to be declared before including it.
UNSAFE_HEADERS_RE='(v2\.0\/standard\/mavlink\.h)'
#UNSAFE_HEADERS_RE='(stm32\.h|arch\/board\/board\.h)'

# Find all submodules.
test -f $BASEDIR/.gitmodules || fatal "No such file: $BASEDIR/.gitmodules"
SUBMODULES=$(grep -A 1 '^\[submodule' $BASEDIR/.gitmodules | grep '^[[:space:]]*path = ' | sed -r -e 's/^[[:space:]]*path = //' | xargs echo)
echo "SUBMODULES=\"$SUBMODULES\""
SUBMODULES_RE="($(echo $SUBMODULES | sed -e 's/ /|/g'))"

# Disable path name expansion (otherwise the find patterns will be expanded against the files in the current working directory).
set -f

EXCLUDE_ARGS=
for excl in $EXCLUDE_FOLDERS; do
	if [ -z "$EXCLUDE_ARGS" ]; then
		EXCLUDE_ARGS="-wholename $BASEDIR/$excl/*"
	else
		EXCLUDE_ARGS+=" -o -wholename $BASEDIR/$excl/*"
	fi
done
for excl in $EXCLUDE_PATTERNS; do
	EXCLUDE_ARGS+=" -o -wholename */$excl/*"
done
INCLUDE_H_ARGS=
for ext in $H_EXTENSIONS $HXX_EXTENSIONS; do
	if [ -z "$INCLUDE_H_ARGS" ]; then
		INCLUDE_H_ARGS="-name *$ext"
	else
		INCLUDE_H_ARGS+=" -o -name *$ext"
	fi
done
INCLUDE_C_ARGS=
for ext in $C_EXTENSIONS $CXX_EXTENSIONS; do
	if [ -z "$INCLUDE_C_ARGS" ]; then
		INCLUDE_C_ARGS="-name *$ext"
	else
		INCLUDE_C_ARGS+=" -o -name *$ext"
	fi
done
# Also exclude all submodules -- because we don't maintain those (are we?).
for subm in $SUBMODULES; do
	if [ -z "$SUBMODULES_ARGS" ]; then
		SUBMODULES_ARGS="-wholename $BASEDIR/$subm/*"
	else
		SUBMODULES_ARGS+=" -o -wholename $BASEDIR/$subm/*"
	fi
done

echo -n "Finding all source files with #include's (excluding submodules and build directory)... "
find $BASEDIR -mindepth 2 -type f ! \( -wholename $BASEDIR/build_* -o $EXCLUDE_ARGS -o $SUBMODULES_ARGS \) \( $INCLUDE_C_ARGS -o $INCLUDE_H_ARGS \) > $TMPDIR/fix_headers_sources
cat "$TMPDIR/fix_headers_sources" | xargs grep -l "$INCLUDE_RE" > $TMPDIR/fix_headers_sources_with_includes
echo "done"
number_of_files=$(sed -n '$=' "$TMPDIR/fix_headers_sources_with_includes")
count=0

echo -n "Finding all submodule header files (excluding stdc++ headers)... "
find $BASEDIR -type f ! \( $EXCLUDE_ARGS \) \( $SUBMODULES_ARGS \) \( $INCLUDE_H_ARGS \) > $TMPDIR/fix_headers_SUBMODULE_HEADERS
echo "done"

echo -n "Finding all header files (excluding stdc++ headers)... "
find $BASEDIR -type f ! \( $EXCLUDE_ARGS \) -wholename $BASEDIR/build_* \( $INCLUDE_H_ARGS \) > $TMPDIR/fix_headers_HEADERS
grep -E "$HEADER_RE" $TMPDIR/fix_headers_sources >> $TMPDIR/fix_headers_HEADERS
cat $TMPDIR/fix_headers_SUBMODULE_HEADERS >> $TMPDIR/fix_headers_HEADERS
echo "done"

echo -n "Finding all include paths... "
for f in `cat $TMPDIR/fix_headers_sources_with_includes`; do grep -E "$INCLUDE_RE" $f | sed -r -e "s%$INCLUDE_RE%%"';s/[">].*//'; done | sort -u | grep -E -v "(/|^)$EXCLUDE_PATTERNS_RE/" > $TMPDIR/fix_headers_include_paths
echo "done"

function include_path()
{
	# If the include path starts with a '.', then it is a local header.
	if [[ $1 =~ ^\. ]]; then return 1; fi
	# If the include path starts with 'platforms/' then it is a local header;
	# added this exception here because not everyone has all build_ directories for all targets installed.
	if [[ $1 =~ platforms/ ]]; then return 1; fi
	# apps.h is generated from apps.h.in.
	if [ $1 = "apps.h" ]; then return 1; fi
	# Treat the following headers from src/platforms/*/include as system header because they replace what is found in nuttx (for posix and qurt).
	if [ $1 = "arch/board/board.h" -o $1 = "crc32.h" -o $1 = "i2c.h" -o $1 = "queue.h" -o $1 = "poll.h" -o $1 = "sys/ioctl.h" ]; then return 2; fi
	# Escape the path for reg.exp. matching.
	PATH_RE=$(echo $1 | sed -e 's/\([+.]\)/\\\1/')
	issubmodule=0;
	islocal=0;
	foo=0
	for includedir in $(grep "/$PATH_RE\$" $TMPDIR/fix_headers_HEADERS | cut -c $striplen-); do
		# If the include directory is NuttX header that was copied to the build directory, then it's still a system file.
		if [[ $includedir/ =~ ^build_.*/NuttX/ ]]; then
			issubmodule=1
		# If the include directory is a submodule, then treat it as a system file.
		elif [[ $includedir/ =~ ^$SUBMODULES_RE/ ]]; then
			issubmodule=1;
		else
			islocal=1
		fi
	done
	if [ $islocal -eq 0 ]; then
		if [ $issubmodule -eq 0 ]; then
			# If an include path can't be found then usually it will be a real system header,
			# however, there are a few (ros related?) files that start with px4... In that
			# case just leave the quotes alone ("px4muorb.h" and several <px4/...>).
			if [[ $1 =~ ^px4 ]]; then return 0; fi
			# While if the include path starts with uORB/topics or topics, and it isn't found,
			# then likely we just don't have a build directory. These should be local though.
			# Same for the generated files mixer_multirotor.generated.h and build_git_version.h.
			if [[ $1 =~ ((/|^)topics/|mixer_multirotor\.generated\.h|build_git_version\.h) ]]; then return 1; fi
		fi
		return 2;
	fi	# Submodule or system header.
	if [ $issubmodule -eq 0 ]; then return 1; fi	# Local.
	# Files that are both local and submodule are simply left alone.
	# These are (at this moment): "battery.h" "common.h" "Matrix.hpp" "mavlink.h" "protocol.h" "pwm.h" "spi.h" "Vector.hpp".
	return 0;
}

# Run the include_path function for each of the files in $TMPDIR/fix_headers_include_paths
echo -n "Determining which headers need to be included with double quotes... "
echo -n > $TMPDIR/fix_headers_quotes
for arg in $(cat "$TMPDIR/fix_headers_include_paths"); do
	include_path $arg
	localsystem=$?
	if [ $localsystem -eq 1 ]; then
		echo "$arg \"$arg\"" >> $TMPDIR/fix_headers_quotes
	elif [ $localsystem -eq 2 ]; then
		echo "$arg <$arg>" >> $TMPDIR/fix_headers_quotes
	fi
done
echo "done"

# Truncate the error log.
echo -n > $TMPDIR/fix_headers_ERROR.log

function print_error
{
	echo
	echo -n "  ";
	echo "*** $1" | tee -a "$TMPDIR/fix_headers_ERROR.log"
	return 1
}

if [ $debug -ne 0 ]; then
	# Debug Line.
	DL='if (cdbl != NR) { printf "\n%u. \"%s\"", NR, $0; cdbl = NR }'
	# Debug Begin.
	DB='if (cdbl != NR) { printf "\n%u. \"%s\" ---> ", NR, $0; cdbl = NR } else printf "; "; printf'
	# Debug End.
	DE=''
else
	DL='#'
	DB='#'
	DE=''
fi
# Error Prefix.
EP='###'

# The main function that is called for each source file.
function fixup_header
{
	count=$((count + 1))
	echo -n "[$((100 * count / number_of_files))%] Fixing headers of $1... "

	# Is this a header?
	echo "$1" | sed -e 's/\.in$/;in/;s/.*\././;s/;in$/.in/' | grep -v -E "$HEADER_RE\$" >/dev/null; is_header=$?
	if [ $debug -ne 0 ]; then echo "is_header = \"$is_header\""; fi

	# Is this C++ source?
	echo "$1" | sed -e 's/.*\././' | grep -v -E $CXXSRC_RE >/dev/null; is_cxxsrc=$?
	if [ $debug -ne 0 ]; then echo "is_cxxsrc = \"$is_cxxsrc\""; fi
	dont_make_cxxsrc=1
	if [ $is_cxxsrc -eq 0 -a $is_header -ne 0 ]; then
		grep -m 1 -q -E "^[[:space:]]*(#[[:space:]]*include[[:space:]]*<$STDCXX_HEADERS_RE>|(template|namespace|class)(\$|[^[:alnum:]_]))" "$1"
		dont_make_cxxsrc=$?
	fi
	if [ $dont_make_cxxsrc -eq 0 ]; then
		is_cxxsrc=1
	fi

	# Current directory.
	curdir=$(dirname "$BASEDIR/$1")

	# Parse the file.
	#
	# Returns an array of line[]'s. The first line is either the first #include, when it is outside
	# any #if*...#endif constructs not counting the header guard if the file is a header. For example:
	#
	#	  // Anything here except #include lines.
	#	  #include <first_include.h>	// <-- first line.
	#
	# Or, the first #if* that contains the first #include line. For example:
	#	  // Anything here except #include lines.
	#	  #ifndef FOO_H // header guard.
	#	  #define FOO_H
	#	  int global = 1; // Anything unknown.
	#	  #ifdef SOMETHING		// <-- first line.
	#	  #if maybe_more
	#	    // anything except #include lines.
	#	  #else
	#	    // anything except #include lines.
	#	    #include <first_include.h>
	#
	# Subsequent line[]'s mark the beginning of a new block, where we have the following blocks:
	# type[]		Description
	type_include=0		# An #include, outside #if*...#endif constructs except a possible header guard.
	type_ifincludeendif=1	# #if*...#endif constructs with #include's.
	type_ifendif=2		# #if*...#endif constructs without #include's.
	type_decls=3		# __BEGIN_DECLS ... __END_DECLS block.
	type_macro=4		# Contiguous #define block.
	type_comment=5		#(Multi-line) comments.
	type_emptyline=6	# Empty lines.
	type_pragmaonce=7	# #pragma once (must be outside any #if*...#endif constructs).
	type_end=8		# The first line of the remainder of the file.
	#
	# However, any block NOT containing one or more #include's (all types > 1) will
	# cause subsequent blocks that do not contain #include's to be ignored,
	# with as result that those blocks will be treated as contiguous code blocks.
	# A comment that is followed by a type that is not to be ignored as such
	# is given the type that follows (which itself is then ignored).
	# Empty lines are ignored unless they appear directly in front of a type
	# that is not to be ignored according to the above rules, where 'previous
	# types' then are the types before the empty line.
	#
	# For example:
	#
	#   >	#include <first_header.h>	# type_include
	#   >	#include <second_header.h>	# type_include
	#   >					# type_emptyline
	#   >	#include <third_header.h>	# type_include
	#   ^	#ifdef FOO			# type_ifendif
	#   |	#define BAR 1
	#   |	#endif				# (detected here)
	#   |								<-- ignored because:
	#   v	#define BAZ(x) x // Do baz				<-- ignored (prev and this type in {type_ifendif, type_macro})
	#   >					# type_emptyline
	#   ^	// This include is important:	# type_comment, but then replaced by type_ifincludeendif
	#   |								<-- ignored because:
	#   |	// more here.						<-- ignored same type
	#   |	#ifdef BAR						<-- "ignored": type_ifincludeendif, but put 3 lines higher.
	#   |	#include <bar.h>
	#   v	#endif				# (detected here)
	#   >					# type_emptyline
	#   >	#include <another.h>		# type_include
	#
	# This script stops parsing at the first not recognized line outside #if*...#endif constructs
	# unless no first line was found yet. It does not attempt overly hard to decode rare constructs,
	# most notably anything with a leading C comment is not recognized and will thus lead to an abort.
	# For example the following lines are not recognized:
	#
	# /* Some comment */ #include <header.h>
	# /* Some comment */ // Another comment.
	#
	# Lines that have a trailing comment are recognized (by ignoring the comment).

	result=$(awk "\
		function add(l, t) {
			# First add always succeeds.
			if (ptr > 0) {
				# An empty line is always added, at first, unless the immediate preceding type is an empty line.
				if (t == $type_emptyline && type[ptr - 1] == $type_emptyline) {
					return;				$DB \"ignored because line %d is also empty.\", line[ptr - 1] $DE
				}
				# A comment is always, added at first, unless the preceding non-empty line type is a comment.
				# Same for #include's.
				if (t == $type_comment && last_none_type == t) {
									$DB \"ignoring because same type as last_none_type (%s)\", type_name[t] $DE
					# Gobble up preceding empty lines.
					if (type[ptr - 1] == $type_emptyline) {
						--ptr;			$DB \"ptr = %d; [43mRemoved type_emptyline @ line %d[0m\", ptr, line[ptr] $DE
					}
					return;

				}
				# {ifendif, macro}'s are collapsed too.
				if ((t == $type_ifendif || t == $type_macro) && (last_nonws_type == $type_ifendif || last_nonws_type == $type_macro)) {
					# Gobble up preceding comments and empty lines.
					while (ptr > 0 && (type[ptr - 1] == $type_emptyline || type[ptr - 1] == $type_comment)) {
						--ptr;			$DB \"ptr = %d; [43mRemoved %s @ line %d[0m\", ptr, type_name[type[ptr]], line[ptr] $DE
					}
					# ptr > 0 here because the first add is never for an empty line or comment.
					last_none_type = type[ptr - 1];	$DB \"last_none_type = %s\", type_name[last_none_type] $DE
					return;
				}
				# type_include and type_pragmaonce and higher are always added.
			}
			if (t == $type_end) {
				# Remove drag.
				while(ptr > 0 && line[ptr - 1] >= l) --ptr;
			}
			# If this type is not an empty line and it was preceded by a comment, then melt it together with that comment.
			else if (t != $type_emptyline && last_none_type == $type_comment) {
				# In this case t cannot be type_comment.
				# Gobble up preceding empty lines.
				if (type[ptr - 1] == $type_emptyline) {
					--ptr;				$DB \"ptr = %d; [43mRemoved type_emptyline @ line %d[0m\", ptr, line[ptr] $DE
				}
				# And replace the comment type.
				--ptr;					$DB \"ptr = %d; [45mreplacing the %s @ line %d[0m\", ptr, type_name[type[ptr]], line[ptr] $DE
				l = line[ptr];

			}
			line[ptr] = l;					$DB \"ptr = %d; [42m%s @ line %d[0m\", ptr, type_name[t], l $DE;
			type[ptr++] = t;
			if (t != $type_emptyline) {
				last_none_type = t;			$DB \"last_none_type = %s\", type_name[last_none_type] $DE
				if (t != $type_comment)
					last_nonws_type = t;
			}
		}

		BEGIN {
			debug = $debug;		# 0: no debug output; non-zero: print debug output.
			header = $is_header;	# 0: do not look for a header guard; 1: treat first #ifndef as header guard.
			in_if = 0;		# The number of nested levels inside #if, #ifdef or #ifndef ... #endif constructs.
			in_if_base = 0;		# 0: no header guard was found (or #pragma once); 1: an #ifndef header guard was found.
			in_decl = 0;		# 0: not inside a __BEGIN_DECLS ... __END_DECLS block; 1: inside such a block.
			found_guard = 0;	# 0: no header guard was found; 1: a header guard was found (including #pragma once).
			base_if = 0;		# The current base-level #if that we are scanning.
			drag = 0;		# The number of lines since the last certainly relevant line (a base-level #include or a base-level #endif containing one of more #includes).
			skipped = 0;		# 0: No #include was encountered in the current (base-level) #if block; 1: one or more #include's encountered in the current base-level #if block.
			in_comment = 0;		# 0: not in a multi-line C comment; 1: in a multi-line C comment.
			cdbl = 0;		# Current debug line.
			error = 0;		# 0: no error occured; 1: an error occured.
			ptr = 0;		# Current pointer into line[] and type[].
			found_comment_end = 0;	# The last line (possibly the current line) that is/was a multi-line C comment termination.
			last_none_type = -1;	# The last non-emptyline type that was added.
			last_nonws_type = -1;	# The last non-whitespace type that was added.
			# For debug purposes:
			type_name[$type_ifendif] = \"type_ifendif\";
			type_name[$type_ifincludeendif] = \"type_ifincludeendif\";
			type_name[$type_decls] = \"type_decls\";
			type_name[$type_comment] = \"type_comment\";
			type_name[$type_emptyline] = \"type_emptyline\";
			type_name[$type_include] = \"type_include\";
			type_name[$type_macro] = \"type_macro\";
			type_name[$type_pragmaonce] = \"type_pragmaonce\";
			type_name[$type_end] = \"type_end\";
		}

		END {
			last_line = NR - drag;
			add(last_line + 1, $type_end);			$DB \"\n\" $DE;
			# Print output.
			if (error || ptr == 0 || last_line < line[0]) {
				print \"error=1\";
				exit
			}
			printf \"lines=\\\"\";
			for (i = 0; i < ptr - 1; ++i)
				printf \"%d \", line[i];
			printf \"%d\\\"; \", line[ptr - 1];
			printf \"types=\\\"\";
			for (i = 0; i < ptr - 1; ++i)
				printf \"%d \", type[i];
			printf \"%d\\\"; \", type[ptr - 1];
			print \"error=0; first_line=\" line[0] \"; last_line=\" last_line
		}

	#======================================================================================================
	# Handle multi-line C comments.

		/$COMMENT_END_RE/ {
			if (in_comment) {
				in_comment = 0;				$DB \"comment end\" $DE;
				found_comment_end = NR;
				sub(/^([^*]|\*+[^*\/])*\*+\//, \"\")	# Remove the tail of the comment.
			}
			# FALL-THROUGH
		}

		{
			if (in_comment) {
				++drag;					$DB \"in comment; drag = %d\", drag $DE;
				next
			}
			found_comment_begin = 0;
			# FALL-THROUGH
		}

		/$COMMENT_BEGIN_RE/ {
			in_comment = 1;					$DB \"comment begin\" $DE;
			found_comment_begin = 1;
			sub(/\/\*([^*]|\*+($|[^*\/]))*$/, \"\")		# Remove the head of the comment so that we'll recognize this as an empty line if it is.
			# FALL-THROUGH
		}

	#======================================================================================================
	# Detect and handle header guard.

		/$PRAGMA_ONCE_RE/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (header && found_guard == 0 && in_if == 0) {
				found_guard = NR;			$DB \"found_guard = %d\", found_guard $DE;
				if (ptr > 0)
					add(NR, $type_pragmaonce)
				next
			}
			print \"\\n$EP $1:\" NR \": unexpected #pragma once\";
			error = 1;
			exit
		}

		/^#ifndef / {
			if (ptr == 0 && header && found_guard == 0 && in_if == 0) {
				found_guard = NR;			$DB \"found_guard = %d\", found_guard $DE;
				in_if = 1;				$DB \"in_if = %d\", in_if $DE;
				in_if_base = 1;				$DB \"in_if_base = %d\", in_if_base $DE;
				next
			}
			# FALL-THROUGH
		}

	#======================================================================================================
	# Detect and handle __BEGIN_DECLS ... __END_DECLS blocks.

		/^[[:space:]]*__BEGIN_DECLS/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (in_decl == 0) {
				in_decl = 1;				$DB \"in_decl = 1\" $DE
				add(NR, $type_decls);
				next
			}
			print \"\\n$EP $1:\" NR \": Nested __BEGIN_DECLS!\";
			error = 1;
			exit
		}

		/^[[:space:]]*__END_DECLS/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (in_decl == 1) {
				in_decl = 0;				$DB \"in_decl = 0\" $DE
				if (skipped) {
					drag = 0;
				} else if (ptr == 1) {
					ptr = 0;			$DB \"erase DECLS block\" $DE
					last_none_type = -1;
				}
				next
			}
			print \"\\n$EP $1:\" NR \": __END_DECLS without matching __BEGIN_DECLS!\";
			error = 1;
			exit
		}

	#======================================================================================================
	# Detect and handle #if ... #endif blocks.

		/$IF_RE/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (in_if == in_if_base && in_decl == 0) {
				skipped = 0;				$DB \"skipped = 0\" $DE;
				base_if = NR;				$DB \"base_if = %d\", NR $DE;
			}
			++in_if;					$DB \"in_if = %d\", in_if $DE;
			next
		}

		/$ENDIF_RE/ {
			--in_if;					$DB \"in_if = %d\", in_if $DE;
			if (in_if < 0) {
				print \"\\n$EP $1:\" NR \": #endif without matching #if!\";
				error = 1;
				exit
			}
			++drag;
			if (in_if == in_if_base && in_decl == 0) {
				if (skipped) {
					drag = 0;
					add(base_if, $type_ifincludeendif);
				} else if (ptr > 0)
					add(base_if, $type_ifendif);
			}						$DB \"drag = %d\", drag $DE;
			# Left header guard?
			if (in_if < in_if_base) {			$DB \"left header guard:\" $DE;
				# assert(in_if == 0 && in_if_base == 1 && header && found_guard)
				exit
			}
			next
		}

	#======================================================================================================
	# Handle #include lines.

		/$INCLUDE_RE/ {
			if (!/\"(\.\/)?mavlink_msg/) {
				# If we're inside a __BEGIN_DECLS ... __END_DECLS block then only certain headers may be included.
				hname = gensub(/^[[:space:]]*#[[:space:]]*include[[:space:]]*[<\"]([^>\"]*)[>\"].*/, \"\\\1\", \"1\");
				cpp_safe = !(hname ~ /$UNSAFE_HEADERS_RE/);
										$DB \"hname = \\\"\" hname \"\\\"; cpp_safe = \" cpp_safe \"; in_decl = \" in_decl \"; is_cxxsrc = $is_cxxsrc\" $DE
				if (in_decl && cpp_safe) {
					print \"\\n$EP $1:\" NR \": including \" hname \" inside a __BEGIN_DECLS ... __END_DECLS block.\";
					error = 1;
					exit
				} else if (!in_decl && !cpp_safe && $is_cxxsrc) {
					print \"\\n$EP $1:\" NR \": including \" hname \" outside a __BEGIN_DECLS ... __END_DECLS block!\";
					error = 1;
					exit
				}
				if (in_if > in_if_base || in_decl) {
					skipped = 1;				$DB \"skipped = 1\" $DE;
				} else {
					drag = 0;				$DB \"drag = 0\" $DE;
					add(NR, $type_include);			$DB \"first_line = %d\", NR $DE;
				}
				next
			}
		}

	#======================================================================================================
	# Ignore #define's, empty lines and lines with just comments.

		/$DEFINE_RE/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (ptr > 0 && in_if == in_if_base && in_decl == 0) {
				add(NR, $type_macro);
			}
			next
		}

		/$EMPTY_LINE_RE/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (ptr > 0 && in_if == in_if_base && in_decl == 0) {
				if (found_comment_begin)
					add(NR, $type_comment);
				else if (found_comment_end != NR)
					add(NR, $type_emptyline);
			}
			next
		}

		/$COMMENT_LINE_RE/ {
			++drag;						$DB \"drag = %d\", drag $DE;
			if (ptr > 0 && in_if == in_if_base && in_decl == 0 && type[ptr - 1] != $type_comment) {
				add(NR, $type_comment);
			}
			next
		}

	#======================================================================================================
	# Handle everything else (unrecognized lines).

		{
			++drag;						$DB \"unknown; drag = %d\", drag $DE;
			if (ptr > 0 && in_if <= in_if_base && in_decl == 0) {
				exit
			}
		}

		" "$BASEDIR/$1")

	# Decode the result.
	vars=$(echo "$result" | tail -n 1)
	error_msg=$(echo "$result" | grep "^$EP " | sed -e 's/^....//')
	if [ $debug -ne 0 ]; then
		len=$(echo "$result" | wc --lines)
		echo "$result" | head -n $((len - 1)) | grep -v "^$EP "	# Debug messages
		echo "vars: $vars"
	fi
	# Evaluate the last line printed in END.
	error=1; eval $vars
	test -z "$error_msg" || print_error "$error_msg" || return
	test $error -eq 0 -a $first_line -gt 0 || print_error "FAILED to find an #include in $1?!" || return
	test $last_line -ge $first_line || print_error "FAILED to find a sensible last line in $1?!" || return

	# Calculate the number of lines starting from the current line.
	# Use sed to count lines, because wc --lines doesn't report the last line when that doesn't end on a new-line, contrary to the fact that tail treats such lines as lines.
	total_lines=$(sed -n '$=' "$BASEDIR/$1")
	if [ $debug -ne 0 ]; then echo "total_lines = \"$total_lines\""; fi

	# Edit the first_line...last_line block.
	# Header files are ordered as follows (lowest value first):
	cat_winsock=0;		# Winsock2.h
	cat_posix_sys=1;	# posix_sys.h or one of the px4_ headers that include it.
	cat_px4=2;		# Other px4_*.h
	cat_local=3;		# "*.h"
	cat_cxx=4;		# <std c++ header>, ie <iostream>
	cat_c=5;		# <c std c++ header>, ie <cstdio>
	cat_system=6;		# <*.h>

	head -n $last_line "$BASEDIR/$1" | tail -n $((last_line - first_line + 1))  | awk "

		function sort_by_type_line_header_type_hname(blk2, v2, blk1, v1) {
			# Return true if blk1 comes before blk2.
			# Move type_include before the rest. Keep the same line order for the rest.
			return (type[blk2] != $type_include && (type[blk1] == $type_include || line[blk1] < line[blk2])) ||
			       (type[blk2] == $type_include && type[blk1] == $type_include &&
				   # If both are include's then put include with a lower header_type first; sort alphabetically for the same header type.
				   (header_type[blk1] < header_type[blk2] || (header_type[blk1] == header_type[blk2] && hname[blk1] < hname[blk2])));
		}

		BEGIN {
			first_line = $first_line;
			split(\"$lines\", line);
			split(\"$types\", type);
			i = 0;
			do {
				line[++i] -= first_line - 1;
			} while(type[i] != $type_end)
			for(b = 0; b < i; ++b) header_type[b] = 100;
			blk = 1;
			n = 0;
			is_cxxsrc = $is_cxxsrc;
			# px4_posix.h includes px4_defines.h includes px4_log.h includes posix_sys.h which must be the first header included.
			sys_val[\"px4_posix.h\"] = 1;
			sys_val[\"px4_defines.h\"] = 2;
			sys_val[\"px4_log.h\"] = 3;
			sys_val[\"posix_sys.h\"] = 4;
			saw_sys_val = 5;		# Didn't see any of the above; otherwise the lowest value of the header seen.
			for(b = 0; b < i; ++b) saw_sys[b] = saw_sys_val;
		}

		END {
			l = asorti(txt, k, \"sort_by_type_line_header_type_hname\");
			for (b = 1; b <= l; ++b) {
				if (type[k[b]] == $type_include && header_type[k[b]] == $cat_posix_sys && saw_sys[k[b]] > saw_sys_val) continue;
				len = length(txt[k[b]]);
				for (n = 0; n < len; ++n) print txt[k[b]][n];
				if (b < l && type[k[b]] == $type_include && type[k[b+1]] != $type_emptyline &&
				    (type[k[b+1]] != $type_include || (header_type[k[b]] != header_type[k[b+1]] && header_type[k[b+1]] != $cat_px4))) {
					printf \"\n\";
				}
			}
		}

		{
			if (NR == line[blk + 1]) {
				++blk;
				n = 0;
			}
		}

		/$INCLUDE_RE/ {
			# Don't use double quotes around standard header names.
			\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*)\\\"$STDC_HEADERS_RE\\.h\\\"/, \"\\\1<\\\2.h>\", \"1\");
			if (is_cxxsrc) {
				\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*)\\\"$STDCXX_HEADERS_RE\\\"/, \"\\\1<\\\2>\", \"1\");
				# Change deprecated C header names to standard C++ header names in C++ source files.
				\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*<)$STDC_HEADERS_RE\\.h>/, \"\\\1c\\\2>\", \"1\");
			}
			# Don't include \"./foo.h\", that is implied, so just include \"foo.h\".
			\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*\\\")\.\//, \"\\\1\", \"1\");
			# Extract the header filename.
			hname[blk] = gensub(/^[[:space:]]*#[[:space:]]*include[[:space:]]*[<\"]([^>\"]*)[>\"].*/, \"\\\1\", \"1\");
			# If the header exists in the directory of the including file, then it is a local header.
			command = sprintf(\"test -e %s/%s\", \"$curdir\", hname[blk]);
			if (system(command) == 0) {
				\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*)[\"<]([^\">]*)[\">]/, \"\\\1\\\"\\\2\\\"\", \"1\");
			} else {
				# Do we know if this is a local file, or a submodule / system header?
				# The grep reg.exp needs \\ (for backslah) and \1 for back reference, thus: \\\1.
				# However we print the grep command using sprintf, so each backslash needs to be escaped once more: \\\\\\1.
				# Finally, this is a bash string and we need to escape each backslash once more to pass it corrently to awk, hence we need twelve backslahes:
				command = sprintf(\"grep '^%s ' '%s' 2>/dev/null\", gensub(/([.+])/, \"\\\\\\\\\\\\1\", \"g\", hname[blk]), \"$TMPDIR/fix_headers_quotes\");
				ret = command | getline result;
				if (ret != 0) {
					result = substr(result, index(result, \" \") + 1);
					\$0 = gensub(/^([[:space:]]*#[[:space:]]*include[[:space:]]*)[\"<][^\">]*[\">]/, \"\\\1\" result, \"1\");
				}
			}
			# Categorise the header.
			if (hname[blk] == \"Winsock2.h\") {
				if (header_type[blk] > $cat_winsock)
					header_type[blk] = $cat_winsock;
			}
			else if (hname[blk] in sys_val) {
				if (header_type[blk] > $cat_posix_sys)
					header_type[blk] = $cat_posix_sys;
				# posix_sys.h is sometimes included within #ifdef __PX4_POSIX ... #endif. The other headers should not be conditional.
				if ((hname[blk] == \"posix_sys.h\" || type[blk] == $type_include)) {
					type[blk] = $type_include;		# Treat #ifdef __PX4_POSIX #include \"posix_sys.h\" #endif as an include for sorting purposes.
					saw_sys[blk] = sys_val[hname[blk]];	# There will be only one include (header name) for this block.
					if (sys_val[hname[blk]] < saw_sys_val)
						saw_sys_val = sys_val[hname[blk]];
				}
				# Use double quotes for these headers.
				\$0 = gensub(/<([[:alnum:]_\/.]*)>/, \"\\\"\\\1\\\"\", \"1\");
			}
			else if (hname[blk] ~ /^(platforms\/px4_|px4_)/) {
				if (header_type[blk] > $cat_px4)
					header_type[blk] = $cat_px4;
				# Use double quotes for these headers.
				\$0 = gensub(/<([[:alnum:]_\/.]*)>/, \"\\\"\\\1\\\"\", \"1\");
			}
			else if (\$0 ~ /^[[:space:]]*#[[:space:]]*include[[:space:]]*\"/) {
				if (header_type[blk] > $cat_local)
					header_type[blk] = $cat_local;
			}
			else if (hname[blk] ~ /^$STDCXX_HEADERS_RE\$/) {
				if (header_type[blk] > $cat_cxx)
					header_type[blk] = $cat_cxx;
			}
			else if (hname[blk] ~ /^c$STDC_HEADERS_RE\$/) {
				if (header_type[blk] > $cat_c)
					header_type[blk] = $cat_c;
			}
			else if (hname[blk] ~ /^$STDC_HEADERS_RE\.h\$/) {
				if (header_type[blk] > $cat_system)
					header_type[blk] = $cat_system;
			}
		}

		{
			# Remove empty lines before #include's.
			if (type[blk] == $type_include) {
				for (i = 1; blk > i && type[blk - i] == $type_emptyline; ++i)
					delete txt[blk - i]
			}
			txt[blk][n++] = \$0;
		}

		" > $TMPDIR/fix_headers_current_block

	# Construct a new file in TMPDIR.
	head -n $((first_line - 1)) "$BASEDIR/$1" > $TMPDIR/fix_headers_current_file

	# Append the editted block.
	cat $TMPDIR/fix_headers_current_block >> $TMPDIR/fix_headers_current_file

	# Append the rest.
	tail -n $((total_lines - last_line)) "$BASEDIR/$1" >> $TMPDIR/fix_headers_current_file

	# Compare original with result.
	if cmp --quiet "$BASEDIR/$1" $TMPDIR/fix_headers_current_file; then
		echo "No change"
	else
		echo "Fixed lines $first_line-$last_line"
		mv $TMPDIR/fix_headers_current_file "$BASEDIR/$1" || fatal "Failed to move $TMPDIR/fix_headers_current_file to $BASEDIR/$1 !?!"
	fi
}

if [ $non_option_arguments -ne 0 ]; then
	fixup_header $1
	exit
fi

# Run the fixup function for each of the files in $TMPDIR/fix_headers_sources_with_includes.
# Strip BASEDIR because we don't know how long that is and it might too much for bash.
for arg in $(cat "$TMPDIR/fix_headers_sources_with_includes" | cut -c $striplen-); do
		fixup_header $arg
done

# Clean up.
if [ $debug -eq 0 -o $# -eq 0 ]; then
	rm "$TMPDIR/fix_headers_sources" "$TMPDIR/fix_headers_sources_with_includes" "$TMPDIR/fix_headers_SUBMODULE_HEADERS" "$TMPDIR/fix_headers_HEADERS" \
	   "$TMPDIR/fix_headers_include_paths" "$TMPDIR/fix_headers_quotes" "$TMPDIR/fix_headers_current_block"
fi

# Print all error messages again at the end.
if [ -s "$TMPDIR/fix_headers_ERROR.log" ]; then
	echo "$0 finished with errors:"
	cat "$TMPDIR/fix_headers_ERROR.log"
else
	echo "SUCCESS"
	rm "$TMPDIR/fix_headers_ERROR.log"
fi
