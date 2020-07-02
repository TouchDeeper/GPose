
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# read specified column of txt
import os
filepath = "../result/"

mu = []
generation_num = []
average_cost = []
best_total_cost = []
with open(filepath + 'report.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        mu_ = line.split()
        numbers_float = map(float, mu_)
        # print(numbers_float)
        generation_num.append(numbers_float[0])
        average_cost.append(numbers_float[1])
        best_total_cost.append(numbers_float[2])
# x = list(range(0, len(mu), 1))
# print(x)
# print(mu)
fig, ax = plt.subplots()
ax.set_yscale("log")
plt.xlabel("iteration")
plt.ylabel("lambda")
plt.plot(generation_num, average_cost, color="r", linestyle="-", marker="*", linewidth=1.0)
plt.plot(generation_num, best_total_cost, color="blue", linestyle="-", marker="+", linewidth=1.0)
plt.show()
plt.savefig("iteration_example.eps",format='eps')

# draw a funciton
# x = np.linspace(1, 60, 100)
# y = 0.5 * np.power(2, np.exp(1-100.0/(100.0+1-x)))
#
# plt.plot(x, y)
#
# plt.title('200')
# plt.xlabel('x')
# plt.ylabel('y')
#
# plt.show()


