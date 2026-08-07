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


def plot_time_series(t, y, ylabel, title: str):
    """
    Plot a time series of data.
    
    Parameters:
    t (list): A list of time values.
    y (list): A list of corresponding data values.
    ylabel (str): The label for the y-axis.
    title (str): The title of the plot.
    """

    plt.figure(figsize=(10, 5))
    plt.plot(t, y)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # Example usage
    import numpy as np

    # Generate some example time series data
    t = np.linspace(0, 10, 100)  # Time from 0 to 10 seconds
    y = np.sin(t)  # Example data: sine wave

    plot_time_series(t, y, ylabel='Amplitude', title='Example Time Series Plot')