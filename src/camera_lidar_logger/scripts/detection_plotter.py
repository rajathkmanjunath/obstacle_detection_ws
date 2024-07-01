#!/usr/bin/env python3

import pandas as pd
from bokeh.plotting import figure, show, output_file
from bokeh.layouts import column
from bokeh.models import DatetimeTickFormatter
from datetime import datetime

'''
Script to generate trigger plots for camera and lidar to evaluate
the performance of the camera against that of the lidar
NOTE: change ${FILE_NAME} to the csv file generated from the run which can be found in /tmp/log.
'''

# Read the CSV file
df = pd.read_csv('${FILE_NAME}.csv', header=None, names=['log'])

# Parse the data
camera_times = []
lidar_times = []

for row in df['log']:
    if 'Camera' in row:
        time_str = row.split(': ')[-1]
        camera_times.append(float(time_str))
    elif 'lidar' in row:
        time_str = row.split(': ')[-1]
        lidar_times.append(float(time_str))

# Convert times to datetime objects
camera_times = [datetime.fromtimestamp(ts) for ts in camera_times]
lidar_times = [datetime.fromtimestamp(ts) for ts in lidar_times]

# Create a DataFrame for plotting
camera_df = pd.DataFrame({'time': camera_times, 'value': [1] * len(camera_times)})
lidar_df = pd.DataFrame({'time': lidar_times, 'value': [1] * len(lidar_times)})

# Determine the earliest time for x-axis alignment
min_time = min(camera_df['time'].min(), lidar_df['time'].min())

# Create Bokeh plots
p1 = figure(title="Camera Detections", x_axis_label='Time', y_axis_label='Detections', x_axis_type='datetime')
p1.scatter(camera_df['time'], camera_df['value'], legend_label='Camera', line_width=2)
p1.xaxis[0].formatter = DatetimeTickFormatter(seconds=["%Y-%m-%d %H:%M:%S"])

p2 = figure(title="Lidar Detections", x_axis_label='Time', y_axis_label='Detections', x_axis_type='datetime')
p2.scatter(lidar_df['time'], lidar_df['value'], legend_label='Lidar', line_width=2)
p2.xaxis[0].formatter = DatetimeTickFormatter(seconds=["%Y-%m-%d %H:%M:%S"])

# Align x-axes start points
p1.x_range.start = min_time
p2.x_range.start = min_time

# Output to static HTML file
output_file("detections.html")

# Show the results
show(column(p1, p2))
