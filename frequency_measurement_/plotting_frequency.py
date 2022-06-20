import sys
from pathlib import Path
import os
import matplotlib.pyplot as plt
import numpy as np

script_dir = Path(__file__).parent
path = os.path.dirname(script_dir)
sys.path.append(path)

# Plot frequency with respect to time with interpolation
fWithInterpol = np.loadtxt(path+'/frequency_measurement_/frequency_with_interpolation.txt', delimiter=',', unpack=True)
line = 50 * np.ones(len(fWithInterpol));


# Plot frequency without respect to time with interpolation
fWithOutInterpol = np.loadtxt(path+'/frequency_measurement_/frequency_without_interpolation.txt', delimiter=',', unpack=True)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica"]})
plt.plot(fWithInterpol)
plt.plot(fWithOutInterpol)
plt.plot(line)
plt.legend(['Frequency with interpolation', 'Frequency without interpolation', 'Baseline'])
plt.title('Frequency')
plt.xlabel('Timestamps')
plt.ylabel('Frequency in Hz')
plt.show()