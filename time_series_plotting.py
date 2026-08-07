import math

import matplotlib.pyplot as plt

def plot_time_series(t, y_list, ylabels, title: str, xlabel: str = "Time [s]", separate_plots: bool = False) -> None:
    """
    Plot a time series of data.
    
    Parameters:
    t (list): A list of time values.
    y_list (list of lists): A list of lists of corresponding data values.
    ylabels (list of strings): The labels for the y-axis.
    title (str): The title of the plot.
    xlabel (str): The label for the x-axis.
    separate_plots (bool): If True, create separate plots for each data series. By default, False (create a single plot with all series).
    """

    series_count = len(y_list)
    if series_count != len(ylabels):
        raise ValueError("The number of y data series must match the number of y labels.")
    
    if separate_plots and series_count > 1:
        # create a separate plot for each series on the same figure

        cols = 2
        rows = math.ceil(series_count / cols)

        fig, axes = plt.subplots(rows, cols, squeeze=False)

        for r in range(rows):
            for c in range(cols):
                index = r * cols + c
                if index < series_count:
                    axes[r, c].plot(t, y_list[index])
                    axes[r, c].set_ylabel(ylabels[index])
                    axes[r, c].set_xlabel(xlabel)
                    axes[r, c].grid()
                else:
                    axes[r, c].axis('off')  # turn off unused subplots
        
        plt.show()
    
    else:
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

    # Generating some example time series data
    t = np.linspace(0, 10, 100)  # Time from 0 to 10 seconds
    y1 = np.sin(t)  # Example data: sine wave
    y2 = np.cos(t)  # Example data: cosine wave
    y3 = np.sin(2 * t)  # Example data: sine wave with double frequency

    plot_time_series(t, [y1, y2, y3], ylabels=['Amplitude', 'Phase', 'Frequency'], title='Example Time Series Plot', separate_plots=True)