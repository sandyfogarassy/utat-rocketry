#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Example data logger that saves DAQ data to a CSV file in real-time"""

import csv
from daq import DAQ

d = DAQ()

with open('data.csv', 'wb') as dataFile:
    writer = csv.writer(dataFile, quoting=csv.QUOTE_ALL)
    writer.writerow(d.source_labels)  # table headers
    while True:
        data = d.data()
        print(data)
        writer.writerow(data)
