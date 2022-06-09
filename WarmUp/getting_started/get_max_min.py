import sys
from pathlib import Path
import os
import matplotlib.pyplot as plt
import numpy as np

script_dir = Path(__file__).parent
path = os.path.dirname(script_dir)
sys.path.append(path)

X = np.loadtxt(path+'/getting_started/max_min.txt', delimiter=',', unpack=True)

m = np.mean(X)
print(m)

plt.plot(X)
plt.title('Measured Input Voltage')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
