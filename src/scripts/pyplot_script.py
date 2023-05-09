import numpy as np
import matplotlib.pyplot as plt


with open('value_function.txt', 'r') as f1:
    data1 = [float(num) for num in f1.read().split()]


with open('policy.txt', 'r') as f2:
    data2 = [float(num) for num in f2.read().split()]


s1 = np.linspace(1, 100, num=100, endpoint=True)
s2 = np.linspace(1, 99, num=99, endpoint=True)

plt.figure()
plt.plot(s1, data1)
plt.show()

plt.figure()
plt.bar(s2, data2)
plt.show()