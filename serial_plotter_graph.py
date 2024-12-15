import plotly.graph_objects as go
from plotly.offline import plot

import serial
import struct
import time

# Replace 'COMx' with the actual COM port of your HC-06 Bluetooth device
# On Windows, it may be something like 'COM4'. On Linux or Mac, it could be '/dev/ttyUSB0' or '/dev/rfcomm0'.
bluetooth_port = 'COM6'  # Update to match your device
baud_rate = 9600  # Baud rate of HC-06 module
num_floats = 9  # Number of floats in the array
float_size = 4  # Size of each float in bytes
byte_order = '<'  # Little-endian ('<' for little-endian, '>' for big-endian)

# Calculate the total number of bytes to read
total_bytes = num_floats * float_size

# Establish Bluetooth serial connection
ser = serial.Serial(bluetooth_port, baud_rate)


def plot_series(data_dict):
    fig = go.Figure()
    
    for name, values in data_dict.items():
        fig.add_trace(go.Scatter(
            y=values,
            mode='lines',
            name=name
        ))
    
    fig.update_layout(title="Measurements",
                    xaxis_title="#",
                    yaxis_title="Value",
                    template="plotly_dark")
    plot(fig, filename="plot.html", auto_open=True)


# Wait for the Bluetooth connection to stabilize
time.sleep(2)

labels = ["r","p","y","aX","aY","aZ","gX","gY","gZ","v"]
max_samples = 350

while True:
    data = {label:[] for label in labels }
    sample_size = 0
    input("Press Enter to plot graph")

    while sample_size < max_samples:
        while ser.in_waiting < total_bytes:
            pass
        recieved_bytes = ser.read(total_bytes)  # Each float is 4 bytes
        values = struct.unpack(f'{byte_order}{num_floats}f', recieved_bytes)
        for label, val in zip(labels,values):
            data[label].append(val)
        sample_size += 1
        print(f"{sample_size}/{max_samples}")
    plot_series(data)
