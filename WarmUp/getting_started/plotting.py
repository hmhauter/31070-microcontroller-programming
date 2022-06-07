import matplotlib.pyplot as plt
import csv
import numpy as np 
 
X = np.loadtxt('data.txt', delimiter=',', unpack=True)
 
plt.plot(X)
plt.title('Measured Input Voltage')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()