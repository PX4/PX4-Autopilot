#! /usr/bin/env python3
"""
function collection for plotting
"""

from typing import Optional, List

from pyulog import ULog
import matplotlib.pyplot as plt
from matplotlib.pyplot import Figure, Axes
from matplotlib.backends.backend_pdf import PdfPages


class DataPlot():
    """
    A plotting class interface. Provides functions such as saving the figure.
    """
    def __init__(
        self, ulog: ULog, plot_title: str, pdf_handle: Optional[PdfPages] = None) -> None:
        """
        Initializes the data plot class interface.
        :param plot_title:
        :param pdf_handle:
        """
        self._ulog = ULog
        self._plot_title = plot_title
        self._pdf_handle = pdf_handle
        self._fig = None
        self._fig_size = (20, 13)

    @property
    def fig(self) -> Figure:
        """
        :return: the figure handle
        """
        if self._fig is None:
            self._create_figure()
        return self._fig

    @property
    def ax(self) -> Axes:
        """
        :return: the axes handle
        """
        return self.fig.gca()

    @property
    def plot_data(self) -> dict:
        """
        returns the plot data. calls _generate_plot_data if necessary.
        :return:
        """
        if self.plot_data is None:
            self._generate_plot_data()
        return self.plot_data

    def plot(self) -> None:
        """
        placeholder for the plotting function. A child class should implement this function.
        :return:
        """

    def _create_figure(self) -> None:
        """
        creates the figure handle.
        :return:
        """
        self._fig = plt.figure(frameon=True, figsize=self._fig_size)
        self._fig.suptitle(self._plot_title)

    def _generate_plot_data(self) -> None:
        """
        placeholder for a function that generates a data table necessary for plotting
        :return:
        """

    def show(self) -> None:
        """
        displays the figure on the screen.
        :return: None
        """
        self.fig.show()


    def save(self) -> None:
        """
        saves the figure if a pdf_handle was initialized.
        :return:
        """

        if self._pdf_handle is not None:
            self._pdf_handle.savefig(figure=self.fig)
        else:
            print('skipping saving to pdf: handle was not initialized.')


class TimeSeriesPlot(DataPlot):
    """
    class for creating bar plots of column statistics.
    """
    def __init__(
        self, ulog: ULog, dataset: str, variable_names: List[str], xlabels: List[str],
        ylabels: List[str], plot_title: str = '', pdf_handle: Optional[PdfPages] = None) -> None:
        """
        initializes a timeseries plot
        :param ulog:
        :param dataset:
        :param variable_names:
        :param xlabels:
        :param ylabels:
        :param plot_title:
        :param pdf_handle:
        """
        super().__init__(ulog, plot_title, pdf_handle=pdf_handle)
        self._dataset = dataset
        self._variable_names = variable_names
        self._xlabels = xlabels
        self._ylabels = ylabels
        

    def plot(self):
        """
        plots the Stats plot.
        :return:
        """

        for i in range(len(self._variables)):
            plt.subplot(2, 1, i + 1)
            for v in self._variables[i]:
                plt.plot(self._ulog.get_dataset(self._dataset).data[v], 'b')
            plt.xlabel(self._xlabels[i])
            plt.ylabel(self._ylabels[i])

        self.fig.tight_layout()