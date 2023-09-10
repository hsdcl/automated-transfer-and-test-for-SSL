from pymeasure.instruments.keithley import Keithley2450
from pymeasure.instruments import list_resources
import pymeasure

class SourceMeter(Keithley2450):
    def __init__(self, adapter):
        super(SourceMeter, self).__init__(adapter)
        self.reset()
        self.use_front_terminals()

    @classmethod
    def find_system(cls):
        system = [x for x in list_resources() if "USB" in x][0]
        adapter = pymeasure.adapters.VISAAdapter(system)
        return adapter

    def apply_current(self, current_range=None, compliance_voltage=0.1):
        assert -1 < current_range < 1
        self.apply_current(current_range, compliance_voltage)

