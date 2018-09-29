#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""DAQ object and resources"""

import nidaqmx


class DAQ:
    """An object representing the physical DAQ connected to this device"""

    def __init__(self):
        self._sources = []
        # get all possible sources dynamically
        for chan_count in range(0,40):
            try:
                self._sources.append(DAQSource('Dev1/ai' + str(chan_count)))
            except nidaqmx.errors.DaqError:
                break

            
    def data(self):
        """Read and return data from all outputs"""
        return list([source.read() for source in self._sources])
    
    def close(self):
        """Remove bindings to the DAQ"""
        for source in self._sources:
            source.close()

    @property
    def source_labels(self):
        """A list of all possible sources of info (outputs) of the physical DAQ"""
        return list([source.label for source in self._sources])
    

class DAQSource:
    """A source of info (output) on the DAQ"""

    def __init__(self, label):
        self._label = label
        self._task = nidaqmx.task.Task()
        self._task.ai_channels.add_ai_voltage_chan(label, terminal_config=nidaqmx.constants.TerminalConfiguration.RSE)

    def close(self):
        """Stop watching this source"""
        self._task.close()

    def read(self):
        """Read the current output from this source"""
        return self._task.read(number_of_samples_per_channel=1)[0]

    @property
    def label(self):
        """The label of this source"""
        return self._label
