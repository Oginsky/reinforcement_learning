import matplotlib.pyplot as plt
from matplotlib import cm


data = {}
with open('mse_on_lambda.txt', 'r') as file:
    for line in file:
        key, value = line.strip().split()
        data[key] = float(value)

x = list(data.keys())
y = list(data.values())

plt.plot(x, y, 'o-')

plt.xlabel('λ')
plt.ylabel('MSE')
plt.title('Зависимость среднеквадратической ошибки от λ (1000 эпизодов)')

plt.show()