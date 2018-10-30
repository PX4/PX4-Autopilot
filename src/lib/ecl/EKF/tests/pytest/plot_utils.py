###############################################################################
#
# Copyright (c) 2017 Estimation and Control Library (ECL). All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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
###############################################################################

"""Plotting utilities for the Python-based tests

@author: Peter Dürr <Peter.Duerr@sony.com>
"""
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import inspect
from contextlib import contextmanager

try:
    # matplotlib don't use Xwindows backend (must be before pyplot import)
    import matplotlib
    matplotlib.use('Agg')

    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    #from matplotlib import pyplot as plt
    #from matplotlib.backends.backend_pdf import PdfPages
    #import seaborn as sns
except ImportError as err:
    print("Cannot import plotting libraries, "
          "please install matplotlib.")
    raise err

# Nice plot defaults
#sns.set_style('darkgrid')
#sns.set_palette('colorblind', desat=0.6)

pp = PdfPages("ecl_EKF_test.pdf")

def quit_figure_on_key(key, fig=None):
    """Add handler to figure (defaults to current figure) that closes it
    on a key press event.
    """
    def quit_on_keypress(event):
        """Quit the figure on key press
        """
        if event.key == key:
            plt.close(event.canvas.figure)

    if fig is None:
        fig = plt.gcf()
    fig.canvas.mpl_connect('key_press_event', quit_on_keypress)


@contextmanager
def figure(name=None, params=None, figsize=None, subplots=None):
    """Setup a figure that can be closed by pressing 'q'
    """
    # As a default, use the name of the calling function as the title of the
    # window
    if name is None:
        # Get name of function calling the context from the stack
        name = inspect.stack()[2][3]
    fig, axes = plt.subplots(*subplots, figsize=figsize)
    #fig.canvas.set_window_title(name)
    #quit_figure_on_key('q', fig)
    yield fig, axes
    if params is not None:
        name += "\n" + repr(params)
    axes[0].set_title(name)
    #plt.show(True)
    pp.savefig()
