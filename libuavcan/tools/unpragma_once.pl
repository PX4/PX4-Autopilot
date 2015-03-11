#!/usr/bin/env perl
###############################################################################
# Purpose:      Remove all occurrences of "#pragma once" from the source tree.
# Usage:        "unpragma-once *.h" or "find . -name \*.h | xargs unpragma-once"
# Author:       Vadim Zeitlin <vadim@tt-solutions.com>
# Licence:      Free Software released under BSD license
# Copyright:    (C) 2011 TT-Solutions SARL
###############################################################################
# Pavel Kirienko <pavel.kirienko@gmail.com>, 2015:
#       Include guard naming adapted to UAVCAN coding style.
###############################################################################

use warnings;
use strict;
use autodie;

use File::Copy qw(move);
use File::Spec ();
use File::Temp ();
use IO::Handle;

sub process_single_file
{
    my $filename = shift;

    my ($volume, $dir, $basename) = File::Spec->splitpath($filename);

    open my $in, '<', $filename;
    my $out = File::Temp->new(DIR => $volume . $dir);

    my $guard = '';
    my $last_was_empty = 0;
    while (<$in>) {
        if (/^#pragma\s+once\s+$/) {
            die "Duplicate #pragma once at $filename:$.\n" if $guard;

            ($guard = uc $filename) =~ s/[\/\.]/_/g;
            $guard .= "_INCLUDED";
            print $out "#ifndef $guard\n";
            print $out "#define $guard\n";
        }
        else {
            $last_was_empty = ($_ =~ /^\s*$/);
            print $out $_
        }
    }

    if ($guard) {
        print $out "\n" unless $last_was_empty;
        print $out "#endif // $guard\n";

        $out->flush();
        move($out->filename, $filename);
    }
}

for (@ARGV) {
    process_single_file $_
}
