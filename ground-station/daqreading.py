import nidaqmx
import nidaqmx.system
import nidaqmx.constants

class DAQ:
    
    def __init__(self):
        self._readers = []
        #configure all outputs
        for source in nidaqmx.system.physical_channel.PhysicalChannel.ai_input_srcs:
            self._readers.append(DAQReader(source))
            
    def data(self):
        dataArr = {}
        for reader in self._readers:
            dataArr[reader.channel] = reader.read()
        return dataArr
    
    def close(self):
        for reader in self._readers:
            reader.close()
    

class DAQReader:

    def __init__(self, channel):
        self._channel = channel
        self.task = nidaqmx.task.Task()
        self.task.ai_channels.add_ai_voltage_chan(channel, terminal_config=nidaqmx.constants.TerminalConfiguration.RSE)

    @property
    def channel(self):
        return self._channel

    def close(self):
        self.task.close()

    def read(self):
        return self.task.read(number_of_samples_per_channel=1)[0]

