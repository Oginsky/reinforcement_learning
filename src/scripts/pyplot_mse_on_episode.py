import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

data = np.loadtxt('learning_curve.txt')
x = data[:, 0]
y = data[:, 1]

norm = plt.Normalize(y.min(), y.max())
colors = cm.coolwarm(norm(y))

for i in range(len(x) - 1):
    plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], c=colors[i])

plt.title('Зависимость среднеквадратической ошибки от номера эпизода (λ = 1.0)')
plt.xlabel('number of episodes')
plt.ylabel('MSE')

plt.show()
