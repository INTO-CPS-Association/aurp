import time
import matplotlib.pyplot as plt
import numpy as np

t_cur = time.time()
t_start = t_cur
t_last = time.time()
ts = 1.0
t_sleep = 0.0001
t = []

while True:
    t_cur = time.time()
    td = t_cur-t_last
    if td >= ts:
        t_last = t_cur
        print(f"sampling, td {td}")
        t.append(t_cur)

    # time.sleep(t_sleep)
    if t_cur - t_start > 20.0:
        break


t = np.array(t)
t = t- t_start
fig, ax = plt.subplots()
ax.plot(t,t)
plt.show()