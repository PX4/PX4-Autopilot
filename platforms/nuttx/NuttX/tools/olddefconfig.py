#!/usr/bin/env python

# Copyright (c) 2018-2019, Ulf Magnusson
# SPDX-License-Identifier: ISC

# Works like 'make olddefconfig', updating an old .config file or creating a
# new one by filing in default values for all new symbols. This is the same as
# picking the default selection for all symbols in oldconfig, or entering the
# menuconfig interface and immediately saving.
#
# The default output filename is '.config'. A different filename can be passed
# in the KCONFIG_CONFIG environment variable.

import kconfiglib


def main():
    kconf = kconfiglib.standard_kconfig()
    kconf.load_config()
    kconf.write_config()


if __name__ == "__main__":
    main()
