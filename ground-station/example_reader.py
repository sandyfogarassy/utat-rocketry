#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Example data logger that saves DAQ data to a CSV file in real-time"""

import csv
from time import time
from daq import DAQ

d = DAQ()

try:
    dataFile = open('data.csv', 'w+')  # overwrite data file if it exists
    writer = csv.writer(dataFile, quoting=csv.QUOTE_ALL)
    writer.writerow(['date'] + d.source_labels)  # table headers
    while True:  # kill/interrupt script to end program
        data = [time()] + d.data()
        print(data)
        writer.writerow(data)
finally:  # close/release everything
    d.close()
    dataFile.close()