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

fig, axs = plt.subplots(1, 1, sharex=True)
color = 'tab:red'
axs.set_ylabel('IAQ ')
# axs.set_xlabel('time (s)')
axs.tick_params(axis='y', labelcolor=color)
axs.plot(ts, iaq, color=color)

ax3 = axs.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax3.set_ylabel('IAQ (0-3)', color=color)  # we already handled the x-label with ax1
ax3.set_ylim(-1, 10)
ax3.plot(ts, iaqAcq, color=color)
# ax3.set_xlabel('Time (s)')
# beautify the x-labels
plt.gcf().autofmt_xdate()

plt.show()
