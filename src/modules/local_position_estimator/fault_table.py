#!/usr/bin/env python
from __future__ import print_function
import pylab as pl
import scipy.optimize
from scipy.stats import chi2

for fa_rate in 1.0/pl.array([1e1, 1e2, 1e4, 1e6, 1e9]):
    print(fa_rate)
    for df in range(1,7):
        f_eq = lambda x: ((1- fa_rate) - chi2.cdf(x, df))**2
        res = scipy.optimize.minimize(f_eq, df)
        assert res['success']
        print('\t', res.x[0])
