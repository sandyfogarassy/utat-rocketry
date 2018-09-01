#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
sys.path.append(os.getcwd())
import daqreading

d = daqreading.DAQ()

while True:
    print(d.data())

