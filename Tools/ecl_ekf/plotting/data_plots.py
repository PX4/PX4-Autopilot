#! /usr/bin/env python3
"""
function collection for plotting
"""

from typing import Optional, List, Tuple, Dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import Figure, Axes
from matplotlib.backends.backend_pdf import PdfPages


def get_min_arg_time_value(
        time_series_data: np.ndarray, data_time: np.ndarray) -> Tuple[int, float, float]:
    """
    :param time_series_data:
    :param data_time:
    :return:
    """
    min_arg = np.argmin(time_series_data)
    min_time = data_time[min_arg]
    min_value = np.amin(time_series_data)
    return (min_arg, min_value, min_time)


def get_max_arg_time_value(
        time_series_data: np.ndarray, data_time: np.ndarray) -> Tuple[int, float, float]:
    """
    :param time_series_data:
    :param data_time:
    :return:
    """
    max_arg = np.argmax(time_series_data)
    max_time = data_time[max_arg]
    max_value = np.amax(time_series_data)
    return max_arg, max_value, max_time


class DataPlot(object):
    """
    A plotting class interface. Provides functions such as saving the figure.
    """
    def __init__(
        self, plot_data: Dict[str, np.ndarray], variable_names: List[List[str]],
        plot_title: str = '', sub_titles: Optional[List[str]] = None,
        x_labels: Optional[List[str]] = None, y_labels: Optional[List[str]] = None,
        y_lim: Optional[Tuple[int, int]] = None, legend: Optional[List[str]] = None,
        pdf_handle: Optional[PdfPages] = None) -> None:
        """
        Initializes the data plot class interface.
        :param plot_title:
        :param pdf_handle:
        """
        self._plot_data = plot_data
        self._variable_names = variable_names
        self._plot_title = plot_title
        self._sub_titles = sub_titles
        self._x_labels = x_labels
        self._y_labels = y_labels
        self._y_lim = y_lim
        self._legend = legend
        self._pdf_handle = pdf_handle
        self._fig = None
        self._ax = None
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
        if self._ax is None:
            self._create_figure()
        return self._ax

    @property
    def plot_data(self) -> dict:
        """
        returns the plot data. calls _generate_plot_data if necessary.
        :return:
        """
        if self._plot_data is None:
            self._generate_plot_data()
        return self._plot_data

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
        self._fig, self._ax = plt.subplots(frameon=True, figsize=self._fig_size)
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

        if self._pdf_handle is not None and self.fig is not None:
            self.plot()
            self._pdf_handle.savefig(figure=self.fig)
        else:
            print('skipping saving to pdf: handle was not initialized.')


    def close(self) -> None:
        """
        closes the figure.
        :return:
        """
        plt.close(self._fig)


class TimeSeriesPlot(DataPlot):
    """
    class for creating multiple time series plot.
    """
    def __init__(
        self, plot_data: dict, variable_names: List[List[str]], x_labels: List[str],
        y_labels: List[str], plot_title: str = '', sub_titles: Optional[List[str]] = None,
        pdf_handle: Optional[PdfPages] = None) -> None:
        """
        initializes a timeseries plot
        :param plot_data:
        :param variable_names:
        :param xlabels:
        :param ylabels:
        :param plot_title:
        :param pdf_handle:
        """
        super().__init__(
            plot_data, variable_names, plot_title=plot_title, sub_titles=sub_titles,
            x_labels=x_labels, y_labels=y_labels, pdf_handle=pdf_handle)

    def plot(self):
        """
        plots the time series data.
        :return:
        """
        if self.fig is None:
            return

        for i in range(len(self._variable_names)):
            plt.subplot(len(self._variable_names), 1, i + 1)
            for v in self._variable_names[i]:
                plt.plot(self.plot_data[v], 'b')
            plt.xlabel(self._x_labels[i])
            plt.ylabel(self._y_labels[i])

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])


class InnovationPlot(DataPlot):
    """
    class for creating an innovation plot.
    """
    def __init__(
        self, plot_data: dict, variable_names: List[Tuple[str, str]], x_labels: List[str],
        y_labels: List[str], plot_title: str = '', sub_titles: Optional[List[str]] = None,
        pdf_handle: Optional[PdfPages] = None) -> None:
        """
        initializes a timeseries plot
        :param plot_data:
        :param variable_names:
        :param xlabels:
        :param ylabels:
        :param plot_title:
        :param sub_titles:
        :param pdf_handle:
        """
        super().__init__(
            plot_data, variable_names, plot_title=plot_title, sub_titles=sub_titles,
            x_labels=x_labels, y_labels=y_labels, pdf_handle=pdf_handle)


    def plot(self):
        """
        plots the Innovation data.
        :return:
        """

        if self.fig is None:
            return

        for i in range(len(self._variable_names)):
            # create a subplot for every variable
            plt.subplot(len(self._variable_names), 1, i + 1)
            if self._sub_titles is not None:
                plt.title(self._sub_titles[i])

            # plot the value and the standard deviation
            plt.plot(
                1e-6 * self.plot_data['timestamp'], self.plot_data[self._variable_names[i][0]], 'b')
            plt.plot(
                1e-6 * self.plot_data['timestamp'],
                np.sqrt(self.plot_data[self._variable_names[i][1]]), 'r')
            plt.plot(
                1e-6 * self.plot_data['timestamp'],
                -np.sqrt(self.plot_data[self._variable_names[i][1]]), 'r')

            plt.xlabel(self._x_labels[i])
            plt.ylabel(self._y_labels[i])
            plt.grid()

            # add the maximum and minimum value as an annotation
            _, max_value, max_time = get_max_arg_time_value(
                self.plot_data[self._variable_names[i][0]], 1e-6 * self.plot_data['timestamp'])
            _, min_value, min_time = get_min_arg_time_value(
                self.plot_data[self._variable_names[i][0]], 1e-6 * self.plot_data['timestamp'])

            plt.text(
                max_time, max_value, 'max={:.2f}'.format(max_value), fontsize=12,
                horizontalalignment='left',
                verticalalignment='bottom')
            plt.text(
                min_time, min_value, 'min={:.2f}'.format(min_value), fontsize=12,
                horizontalalignment='left',
                verticalalignment='top')

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])


class ControlModeSummaryPlot(DataPlot):
    """
    class for creating a control mode summary plot.
    """

    def __init__(
            self, data_time: np.ndarray, plot_data: dict, variable_names: List[List[str]],
            x_label: str, y_labels: List[str], annotation_text: List[str],
            additional_annotation: Optional[List[str]] = None, plot_title: str = '',
            sub_titles: Optional[List[str]] = None,
            pdf_handle: Optional[PdfPages] = None) -> None:
        """
        initializes a timeseries plot
        :param plot_data:
        :param variable_names:
        :param xlabels:
        :param ylabels:
        :param plot_title:
        :param sub_titles:
        :param pdf_handle:
        """
        super().__init__(
            plot_data, variable_names, plot_title=plot_title, sub_titles=sub_titles,
            x_labels=[x_label]*len(y_labels), y_labels=y_labels, pdf_handle=pdf_handle)
        self._data_time = data_time
        self._annotation_text = annotation_text
        self._additional_annotation = additional_annotation


    def plot(self):
        """
        plots the control mode data.
        :return:
        """

        if self.fig is None:
            return

        colors = ['b', 'r', 'g', 'c']

        for i in range(len(self._variable_names)):
            # create a subplot for every variable
            plt.subplot(len(self._variable_names), 1, i + 1)
            if self._sub_titles is not None:
                plt.title(self._sub_titles[i])

            for col, var in zip(colors[:len(self._variable_names[i])], self._variable_names[i]):
                plt.plot(self._data_time, self.plot_data[var], col)

            plt.xlabel(self._x_labels[i])
            plt.ylabel(self._y_labels[i])
            plt.grid()
            plt.ylim(-0.1, 1.1)

            for t in range(len(self._annotation_text[i])):

                _, _, align_time = get_max_arg_time_value(
                    np.diff(self.plot_data[self._variable_names[i][t]]), self._data_time)
                v_annot_pos = (t+1.0)/(len(self._variable_names[i])+1) # vert annotation position

                if np.amin(self.plot_data[self._variable_names[i][t]]) > 0:
                    plt.text(
                        align_time, v_annot_pos,
                        'no pre-arm data - cannot calculate {:s} start time'.format(
                            self._annotation_text[i][t]), fontsize=12, horizontalalignment='left',
                        verticalalignment='center', color=colors[t])
                elif np.amax(self.plot_data[self._variable_names[i][t]]) > 0:
                    plt.text(
                        align_time, v_annot_pos, '{:s} at {:.1f} sec'.format(
                            self._annotation_text[i][t], align_time), fontsize=12,
                        horizontalalignment='left', verticalalignment='center', color=colors[t])

            if self._additional_annotation is not None:
                for a in range(len(self._additional_annotation[i])):
                    v_annot_pos = (a + 1.0) / (len(self._additional_annotation[i]) + 1)
                    plt.text(
                        self._additional_annotation[i][a][0], v_annot_pos,
                        self._additional_annotation[i][a][1], fontsize=12,
                        horizontalalignment='left', verticalalignment='center', color='b')

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])


class CheckFlagsPlot(DataPlot):
    """
    class for creating a control mode summary plot.
    """

    def __init__(
            self, data_time: np.ndarray, plot_data: dict, variable_names: List[List[str]],
            x_label: str, y_labels: List[str], y_lim: Optional[Tuple[int, int]] = None,
            plot_title: str = '', legend: Optional[List[str]] = None,
            sub_titles: Optional[List[str]] = None, pdf_handle: Optional[PdfPages] = None,
            annotate: bool = False) -> None:
        """
        initializes a timeseries plot
        :param plot_data:
        :param variable_names:
        :param xlabels:
        :param ylabels:
        :param plot_title:
        :param sub_titles:
        :param pdf_handle:
        """
        super().__init__(
            plot_data, variable_names, plot_title=plot_title, sub_titles=sub_titles,
            x_labels=[x_label]*len(y_labels), y_labels=y_labels, y_lim=y_lim, legend=legend,
            pdf_handle=pdf_handle)
        self._data_time = data_time
        self._b_annotate = annotate


    def plot(self):
        """
        plots the control mode data.
        :return:
        """

        if self.fig is None:
            return

        colors = ['b', 'r', 'g', 'c', 'k', 'm']

        for i in range(len(self._variable_names)):
            # create a subplot for every variable
            plt.subplot(len(self._variable_names), 1, i + 1)
            if self._sub_titles is not None:
                plt.title(self._sub_titles[i])

            for col, var in zip(colors[:len(self._variable_names[i])], self._variable_names[i]):
                plt.plot(self._data_time, self.plot_data[var], col)

            plt.xlabel(self._x_labels[i])
            plt.ylabel(self._y_labels[i])
            plt.grid()
            if self._y_lim is not None:
                plt.ylim(self._y_lim)

            if self._legend is not None:
                plt.legend(self._legend[i], loc='upper left')

            if self._b_annotate:
                for col, var in zip(colors[:len(self._variable_names[i])], self._variable_names[i]):
                    # add the maximum and minimum value as an annotation
                    _, max_value, max_time = get_max_arg_time_value(
                        self.plot_data[var], self._data_time)
                    mean_value = np.mean(self.plot_data[var])

                    plt.text(
                        max_time, max_value,
                        'max={:.4f}, mean={:.4f}'.format(max_value, mean_value), color=col,
                        fontsize=12, horizontalalignment='left', verticalalignment='bottom')

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])
