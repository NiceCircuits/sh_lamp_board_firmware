# digital PLL simulation

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

f = 49.8
N = 3200
Ncycles = 200
t_end = Ncycles / f
f_raise = 0.001
f_drop = 0.0005

fpll_min = 49.5
fpll_max = 50.5
delta_filt_coef = 128
delta_max = 65536
delta_filt = 1 / delta_filt_coef

time_arr = []
signal_arr = []
pll_arr = []
xor_arr = []
fpll_arr = []
delta_arr = []
t = 0
last_pll_change = 0
pll = 0
delta = 0

fpll = fpll_min


def get_sample(t):
    return np.sin(2 * np.pi * f * (1 + t / t_end * f_raise - ((t > 10) and (t < 15)) * f_drop) * t)


while 1:
    signal = get_sample(t)
    if t >= (last_pll_change + 1 / fpll / 2):
        pll = 1 - pll
        last_pll_change += 1 / fpll / 2
    xor = (signal > 0) != pll

    pll_arr.append(pll)
    time_arr.append(t)
    signal_arr.append(signal)
    xor_arr.append(xor)
    fpll_arr.append(fpll)

    delta = int(delta * (1 - delta_filt) + xor * delta_max * delta_filt)
    delta_arr.append(delta)
    fpll = min(max(fpll_min, fpll_max - (fpll_max - fpll_min) * delta / (delta_max)), fpll_max)
    t += 1 / N
    if t > t_end:
        break

plt.clf()
plt.subplot(411)
plt.plot(time_arr, signal_arr)
plt.plot(time_arr, pll_arr)
plt.subplot(412)
plt.plot(time_arr, xor_arr)
plt.subplot(413)
plt.plot(time_arr, delta_arr)
plt.subplot(414)
plt.plot(time_arr, fpll_arr)
plt.show()
