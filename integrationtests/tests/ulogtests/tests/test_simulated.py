"""
Tests that only make sense for simulated flights.
"""
from uloganalysis import attitudeanalysis as attanl
from uloganalysis import positionanalysis as posanl
from uloganalysis import ulogconv
from uloganalysis import loginfo
import pyulog
import os
import numpy as np
import pytest


# class TestSomething:
#
    # def test_1(self, filepath):
    #     topics = [
    #         "topic1",
    #         "topic2",
    #     ]
    #    setup_dataframe(self, filepath)
#        assert True
#    def test_2(self, filepath):
    #     topics = [
    #         "topic1",
    #         "topic2",
    #     ]
        # setup_dataframe(self, filepath)
#        assert True
