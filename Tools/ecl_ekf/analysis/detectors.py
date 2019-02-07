#! /usr/bin/env python3
"""
detectors
"""
from typing import NamedTuple, Optional
import numpy as np
import pandas as pd
from pyulog import ULog


class PreconditionError(Exception):
    """
    a class for a Precondition Error
    """


class Airtime(NamedTuple):
    """
    Airtime struct.
    """
    take_off: float
    landing: float


class InAirDetector(object):
    """
    in air detector
    """

    def __init__(self, ulog: ULog, min_flight_time_seconds: float = 0.0,
                 in_air_margin_seconds: float = 0.0) -> None:

        self._ulog = ulog
        self._min_flight_time_seconds = min_flight_time_seconds
        self._in_air_margin_seconds = in_air_margin_seconds

        try:
            self._vehicle_land_detected = ulog.get_dataset('vehicle_land_detected').data
            self._landed = self._vehicle_land_detected['landed']
        except:
            self._in_air = []
            raise PreconditionError(
                'InAirDetector: Could not find vehicle land detected message and/or landed field'
                ' and thus not find any airtime.')

        self._log_start = self._ulog.start_timestamp / 1.0e6

        self._in_air = self._detect_airtime()


    def _detect_airtime(self) -> Optional[Airtime]:
        """
        detects the airtime take_off and landing of a ulog.
        :return: a named tuple of ('Airtime', ['take_off', 'landing']) or None.
        """

        # test whether flight was in air at all
        if (self._landed > 0).all():
            print('InAirDetector: always on ground.')
            return []

        # find the indices of all take offs and landings
        take_offs = np.where(np.diff(self._landed) < 0)[0].tolist()
        landings = np.where(np.diff(self._landed) > 0)[0].tolist()

        # check for start in air.
        if len(take_offs) == 0 or ((len(landings) > 0) and (landings[0] < take_offs[0])):

            print('Started in air. Take first timestamp value as start point.')
            take_offs = [-1] + take_offs

        # correct for offset: add 1 to take_off list
        take_offs = [take_off + 1 for take_off in take_offs]
        if len(landings) < len(take_offs):
            print('No final landing detected. Assume last timestamp is landing.')
            landings += [len(self._landed) - 2]
        # correct for offset: add 1 to landing list
        landings = [landing + 1 for landing in landings]

        assert len(landings) == len(take_offs), 'InAirDetector: different number of take offs' \
                                                ' and landings.'

        in_air = []
        for take_off, landing in zip(take_offs, landings):
            if (self._vehicle_land_detected['timestamp'][landing] / 1e6 -
                    self._in_air_margin_seconds) - \
                    (self._vehicle_land_detected['timestamp'][take_off] / 1e6 +
                     self._in_air_margin_seconds) >= self._min_flight_time_seconds:
                in_air.append(Airtime(
                    take_off=(self._vehicle_land_detected['timestamp'][take_off] -
                              self._ulog.start_timestamp) / 1.0e6 + self._in_air_margin_seconds,
                    landing=(self._vehicle_land_detected['timestamp'][landing] -
                             self._ulog.start_timestamp) / 1.0e6 - self._in_air_margin_seconds))
        if len(in_air) == 0:
            print('InAirDetector: no airtime detected.')

        return in_air

    @property
    def airtimes(self):
        """
        airtimes
        :return:
        """
        return self._in_air

    @property
    def take_off(self) -> Optional[float]:
        """
        first take off
        :return:
        """
        return self._in_air[0].take_off if self._in_air else None

    @property
    def landing(self) -> Optional[float]:
        """
        last landing
        :return: the last landing of the flight.
        """
        return self._in_air[-1].landing if self._in_air else None

    @property
    def log_start(self) -> Optional[float]:
        """
        log start
        :return: the start time of the log.
        """
        return self._log_start

    def get_take_off_to_last_landing(self, dataset) -> list:
        """
        return all indices of the log file between the first take_off and the
        last landing.
        :param dataset:
        :return:
        """
        try:
            data = self._ulog.get_dataset(dataset).data
        except:
            print('InAirDetector: {:s} not found in log.'.format(dataset))
            return []

        if self._in_air:
            airtime = np.where(((data['timestamp'] - self._ulog.start_timestamp) / 1.0e6 >=
                                self._in_air[0].take_off) & (
                                    (data['timestamp'] - self._ulog.start_timestamp) /
                                    1.0e6 < self._in_air[-1].landing))[0]

        else:
            airtime = []

        return airtime

    def get_airtime(self, dataset) -> list:
        """
        return all indices of the log file that are in air
        :param dataset:
        :return:
        """
        try:
            data = self._ulog.get_dataset(dataset).data
        except:
            raise PreconditionError('InAirDetector: {:s} not found in log.'.format(dataset))

        airtime = []
        if self._in_air is not None:
            for i in range(len(self._in_air)):
                airtime.extend(np.where(((data['timestamp'] - self._ulog.start_timestamp) / 1.0e6 >=
                                         self._in_air[i].take_off) & (
                                             (data['timestamp'] - self._ulog.start_timestamp) /
                                             1.0e6 < self._in_air[i].landing))[0])

        return airtime

    @staticmethod
    def get_airtime_from_meta(meta_tuple, sample_rate):
        """
        function retruning a na array of 1 or 0 showing if the input trajectory was in air at
        that point.
        :param meta_tuple: tuple containing the meta data
        :param sample_rate: sampling time of the trajectory
        :return:
        """

        def overwrite_slices(traj, slices):
            """
            overwrite slices that are in air with 1
            :param traj: trajectory that should be overwritten at parts that are in air
            :param slices: parts that are in air
            :return:
            """
            for sli in slices:
                traj[sli[0]:sli[1]] = np.ones(sli[1]-sli[0], dtype=int)

            return traj

        slices = []
        log_len = int(meta_tuple.airtimes[-1][1]*1e6/sample_rate) # in ns
        zero_array = np.zeros(log_len, dtype=int)
        for i, at in enumerate(meta_tuple.airtimes):
            slices.append([int(at.take_off*1e6/sample_rate), int(at.landing*1e6/sample_rate)])

        in_air = overwrite_slices(zero_array, slices)

        return in_air

    @staticmethod
    def filter_take_offs(meta_tuple, datasets, log_frames):

        """
        returns a list of log frames being in air.
        :param meta_tuple: contains flight modes, and their start times
        :param datasets: the datasets the model should train on
        :param log_frames: sequences containing the flight modes specified
        :return: log_frames_in_air
        """
        log_frames_in_air = []
        in_air_times = meta_tuple.airtimes

        for in_air_time in in_air_times:
            frames = log_frames.copy()
            for dataset in datasets:
                frames[dataset] = frames[dataset].loc[pd.Timestamp(
                    in_air_time.take_off, unit='s'):pd.Timestamp(in_air_time.landing, unit='s')]
            log_frames_in_air.append(frames)

        return log_frames_in_air