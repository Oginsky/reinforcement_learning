import numpy as np
from matplotlib import pyplot as plt

V = {}
w = 0
h = 0

# read data
with open('cliff_policy.txt', 'r') as file:
    w_h = file.readline()
    state_w, state_h = w_h.strip().split(' ')
    w = int(state_w)
    h = int(state_h)

    for line in file:
        state_first, state_second, value = line.strip().split(' ')
        state = (int(state_first), int(state_second))
        V[state] = str(value)


# show data
fig, ax = plt.subplots()
ax.set_xticks(np.arange(0, w, 1))
ax.set_yticks(np.arange(0, h, 1))
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.axis('off')

for i in range(h):
    for j in range(w):
        ax.text(j, i, V[(j, i)], ha='right', va='top')

plt.show()
