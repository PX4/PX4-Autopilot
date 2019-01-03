############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include(CheckCSourceCompiles)

#=============================================================================
#
#	check_i2c_dev_h
#
#	This function checks <linux/i2c-dev.h> header for i2c_msg structure
#   definition.
#
#	Usage:
#       check_i2c_dev_h(<variable>)
#
#   Output:
#       The provided variable will be set to TRUE if i2c-dev.h contains
#       definition for the i2c_msg structure
#
function(check_i2c_dev_h HAS_I2C_MSG)
    # We intentionally don't include <linux/i2c.h> so that compilation will
    # fail if i2c-dev is sane
    check_c_source_compiles("
                    #include <linux/i2c-dev.h>
                    #include <sys/ioctl.h>

                    int main()
                    {
                        struct i2c_msg msg;
                        struct i2c_rdwr_ioctl_data data;
                        msg.addr = 0x0;
                        msg.flags = 0x0;
                        msg.len = 0;
                        msg.buf = NULL;
                        data.msgs = &msg;
                        data.nmsgs = 1;
                        ioctl(0, I2C_RDWR, (unsigned long)&data);
                        return 0;
                    }
            " I2C_DEV_HAS_I2C_MSG)
    set(${HAS_I2C_MSG} ${I2C_DEV_HAS_I2C_MSG} PARENT_SCOPE)
endfunction()
