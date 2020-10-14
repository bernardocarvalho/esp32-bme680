#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct  4 16:27:43 2020

@author: bernardo
"""

import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
from datetime import datetime, timezone

ts = []
p = []
tmp = []
iaq = []
iaqAcq = []
gRes = []
hum = []
cO2 = []
voc = []

staticIaq = []

if len(sys.argv) > 1:
    filename = str(sys.argv[1])
else:
    filename = 'bme680_data.csv'

with open(filename, 'r') as csvfile:
    data = csv.reader(csvfile, delimiter=',')
    for row in data:
        ts.append(datetime.fromtimestamp(int(row[0]), timezone.utc))
        p.append(float(row[2]))
        gRes.append(float(row[3]))
        iaq.append(float(row[4]))
        iaqAcq.append(int(row[5]))
        tmp.append(float(row[6]))
        hum.append(float(row[7]))
        cO2.append(float(row[9]))
        voc.append(float(row[10]))

fig, axs = plt.subplots(2, 2, sharex=True)

color = 'tab:red'
line1, = axs[0, 0].plot(ts,p, color=color)

axs[0, 0].set_ylabel('Pressure Pa')

axs[0,0].set_title("BME680 data")
axs[0,0].set_ylim(1.0e5, 1.03e5)
axs[0,0].tick_params(axis='y', labelcolor=color)

ax2 = axs[0,0].twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Temp (deg)', color=color)  # we already handled the x-label with ax1
ax2.plot(ts, tmp, color=color)
ax2.tick_params(axis='y', labelcolor=color)

color = 'tab:red'
axs[1,0].set_ylabel('IAQ ')
axs[1,0].set_xlabel('Time (s)')
axs[1,0].tick_params(axis='y', labelcolor=color)
axs[1,0].plot(ts,iaq, color=color)

ax3 = axs[1,0].twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax3.set_ylabel('Temp (deg)', color=color)  # we already handled the x-label with ax1
ax3.plot(ts, iaqAcq, color=color)
ax3.set_xlabel('Time (s)')

color = 'tab:red'
axs[0,1].set_ylabel('humidity')
axs[0,1].set_xlabel('Time (s)')
axs[0,1].tick_params(axis='y', labelcolor=color)
axs[0,1].plot(ts, hum, color=color)

color = 'tab:blue'
ax4 = axs[0,1].twinx()  # instantiate a second axes that shares the same x-axis
ax4.set_ylabel('gasResistance ', color=color)  # we already handled the x-label with ax1
ax4.plot(ts, gRes, color=color)

color = 'tab:red'
axs[1,1].set_ylabel('VOC')
#axs[1,1].set_xlabel('Time (s)')
axs[1,1].tick_params(axis='y', labelcolor=color)
axs[1,1].plot(ts, voc, color=color)

color = 'tab:blue'
ax5 = axs[1,1].twinx()  # instantiate a second axes that shares the same x-axis
ax5.set_ylabel('CO_2 ', color=color)  # we already handled the x-label with ax1
ax5.plot(ts, cO2, color=color)
ax5.set_xlabel('Time (s)')

plt.show()
