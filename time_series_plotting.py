import math

import matplotlib.pyplot as plt

# get the time series data passed as arguments
# TODO: check if the data is in the correct format, if not, convert it to the correct format
# TODO: check attitude and orbit modules
"""
plot_orbit(t: List[float], kep: List[KeplerianElements])

Absent the actual simulation runtime to generate the data, I would just manually 
create some short timeseries that have the right data shape in order to test that everything looks as you expect. 
The physics aren't important to the plotting so just some random sine waves or something would be fine.
"""

# plotting
# x-axis: time ... what representation? system time? or just seconds since the program started

# y-axis: always a float


def plot_time_series(t, y_list, ylabels, title: str, xlabel: str = "Time [s]", separate_plots: bool = False) -> None:
    """
    Plot a time series of data.
    
    Parameters:
    t (list): A list of time values.
    y_list (list of lists): A list of lists of corresponding data values.
    ylabels (list): The labels for the y-axis.
    title (str): The title of the plot.
    xlabel (str): The label for the x-axis.
    separate_plots (bool): If True, create separate plots for each data series. By default, False (create a single plot with all series).
    """

    series_count = len(y_list)
    if series_count != len(ylabels):
        raise ValueError("The number of y data series must match the number of y labels.")
    

    # create a single plot with all series
    plt.figure()
    for index in range(series_count):
        plt.plot(t, y_list[index], label=ylabels[index])
    
    plt.xlabel(xlabel)
    plt.ylabel("Value")
    plt.title(title)
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    # Example usage
    import numpy as np

    # Generate some example time series data
    t = np.linspace(0, 10, 100)  # Time from 0 to 10 seconds
    y1 = np.sin(t)  # Example data: sine wave
    y2 = np.cos(t)  # Example data: cosine wave
    y3 = np.sin(2 * t)  # Example data: sine wave with double frequency

    plot_time_series(t, [y1, y2, y3], ylabels=['Amplitude', 'Phase', 'Frequency'], title='Example Time Series Plot', separate_plots=False)