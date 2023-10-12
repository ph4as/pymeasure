#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2017 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

from pymeasure.instruments import Instrument
from pymeasure.instruments.validators import truncated_range
from pymeasure.instruments.validators import strict_discrete_set
from pymeasure.instruments.validators import strict_range
from pymeasure.adapters import PrologixAdapter

from io import StringIO
import numpy as np
import pandas as pd
import time


class HP8970B(Instrument):
    """ Represents the HP8970B Noise Figure Meter
    and provides a high-level interface for taking measurements of
    high-frequency spectrums
    """

    FREQ_MIN = 10
    FREQ_MAX = 20000  # max of HP8341B
    FINE_TUNE_TIME = 7  # 7 seconds for 1 frequency tune

    DATA_READY = 0x1
    CALIBRATE_COMPLETE = 0x2
    FINE_TUNE_NEEDED = 0x2
    FINE_TUNE_COMPLETE = 0x10

    isTuned = False
    isCalibrated = False
    calibrate_step_started = False
    DEBUG = False

    def dbgprint(self, *args):
        if self.DEBUG:
            print(args[0])

    # Wrapper functions for the Adapter object
    def ask(self, command):
        """ Writes the command to the instrument through the adapter
        and returns the read response.

        :param command: command string to be sent to the instrument
        :returns: String ASCII response of the instrument
        """

        self.dbgprint("ask: " + command)
        self.write(command)
        data = self.read()
        self.dbgprint("data: " + data)
        return data

    def ask_prologix(self, command):
        """ Writes the command to the instrument through the adapter
        and returns the read response.

        :param command: command string to be sent to the instrument
        :returns: Raw response of the instrument
        """

        self.dbgprint("ask_prologix: " + command)
        super(HP8970B, self).write(command)
        return super(HP8970B, self).read_raw()

    def write(self, command):
        """ Writes the command to the instrument through the adapter.

        :param command: command string to be sent to the instrument
        """
        self.dbgprint("write: " + command)
        super(HP8970B, self).write(command)

    def read(self):
        """ Reads the response of the instrument until timeout

        :returns: String ASCII response of the instrument
        """
        self.dbgprint("read: ")
        return super(HP8970B, self).read()

    def read_measurement(self, timeout=5000):
        """ Reads the response of the instrument until timeout

        :returns: String ASCII response of the instrument
        """
        if (self.adapter == PrologixAdapter):
            self.ask_prologix("++spoll")
        self.trigger = True
        if (self.wait_for_srq(timeout, self.DATA_READY)[0] == 0):
            data = super(HP8970B, self).read()
            # print("read: ",end='')
            # print(data)
        else:
            data = ""
        if (self.adapter == PrologixAdapter):
            self.ask_prologix("++spoll")
        # return b"\n".join(self.connection.readlines()).decode()
        data = data.split(',')
        return (float(data[0]) / 1e6, float(data[1]), float(data[2]))

    def set_srq_mask(self, stb_mask=0, estb_mask=0):
        self.write("Q0")  # Disable all
        if (estb_mask):  # Enable Extended STB if mask set
            stb_mask = stb_mask | 0x80
        self.write("RM" + str(stb_mask) + "EN")  # Enable STB mask
        self.write("RE" + str(estb_mask) + "EN")  # Enable Extended STB mask
        self.clear_stb = True

    def wait_for_srq(self, timeout=5000, stb_mask=0, estb_mask=0):
        start_time = time.perf_counter()
        OSTB = (0, 0)
        while True:
            # print("time: " + str(time.perf_counter() - start_time))
            STB = int(self.adapter.connection.stb)
            if (STB != 0):
                self.dbgprint("")
                self.dbgprint(STB)
            if (estb_mask and (STB & 0x80)):
                timed_out = 0
                # print("Extended status")
                OSTB = self.adapter.binary_values("OS", 0, np.uint8)
                if (OSTB[1] != 0):
                    self.dbgprint(OSTB)
                if (OSTB[1] & estb_mask):
                    break
                else:
                    continue
            if STB & stb_mask:
                timed_out = 0
                # print("STB matches mask")
                # print(self.read())
                break
            if (time.perf_counter() - start_time) > timeout:
                timed_out = 1
                self.dbgprint("")
                break
            time.sleep(0.2)
            # self.dbgprint(".", end='')
        print("time taken: " + str(time.perf_counter() - start_time))
        return (timed_out, STB, OSTB[1])

    def set_float_value(val):
        global aunits
        val = "{val:g}{unit}".format(val=val, unit=aunits)
        # print("float value set to ",val)
        return val

    def values(self, command, separator=',', cast=float):
        """ Writes a command to the instrument and returns a list of formatted
        values from the result

        :param command: SCPI command to be sent to the instrument
        :param separator: A separator character to split the string into a list
        :param cast: A type to cast the result
        :returns: A list of the desired type, or strings where the casting fails
        """
        results = str(self.ask(command)).strip()
        results = results.split(separator)
        for i, result in enumerate(results):
            try:
                if cast == bool:
                    # Need to cast to float first since results are usually
                    # strings and bool of a non-empty string is always True
                    results[i] = bool(float(result))
                else:
                    results[i] = cast(result)
            except Exception:
                pass  # Keep as string
        return results

    def fine_tune(self):
        timeout = self.FINE_TUNE_TIME * self.frequencies.size + 10  # calculate total time required
        self.set_srq_mask(0, self.FINE_TUNE_COMPLETE)
        # input("Press Enter to continue...")
        STB = int(self.adapter.connection.stb)  # Clear STB
        # print(STB)
        # OSTB = nfm.adapter.binary_values("OS", 0, np.uint8)
        # print(OSTB)
        self.read()  # clear data
        self.write("PF")
        self.wait_for_srq(timeout, 0, self.FINE_TUNE_COMPLETE)
        self.isTuned = True

    def calibrate(self):
        timeout = 10 + (1.5 + 0.21 * self.smoothing_factor) * self.frequencies.size
        print("calculated timeout: " + str(timeout))
        self.hold = False
        self.set_srq_mask(self.CALIBRATE_COMPLETE)
        self.write("CA")
        self.wait_for_srq(timeout, self.CALIBRATE_COMPLETE)
        self.write("M2")  # Switch to calibrated readings
        self.isCalibrated = True

    def calibrate_step(self, timeout=120):
        if (self.calibrate_step_started != True):
            self.calibrate_step_started = True
            self.hold = True
            self.write("H2")
            self.set_srq_mask(self.CALIBRATE_COMPLETE | self.DATA_READY)
            self.write("CA")
        self.trigger = True
        (timedout, stb, estb) = self.wait_for_srq(timeout, self.CALIBRATE_COMPLETE | self.DATA_READY)
        self.dbgprint("returned stb: " + str(stb))
        data = self.read()
        data = data.split(',')
        if (stb & self.CALIBRATE_COMPLETE):
            self.calibrate_step_started = False
            self.isCalibrated = True
            self.write("M2")  # Switch to calibrated readings
        return (float(data[0]) / 1e6, float(data[1]), float(data[2]), self.calibrate_step_started)

    def set_measurement_mode(self, mode):
        mode_command = "E" + str(mode)
        self.write(mode_command)
        # force update to previous value as mode update can change that
        self.set_frequency_step(self.shadow_frequency_step, True)
        self.measurement_mode = mode

    data_output_full = Instrument.setting(
        "%s", "Left, INSERTION GAIN, NOISE FIGURE displays",
        map_values=True,
        values={True: 'H1', False: 'H0'}
    )

    hold = Instrument.setting(
        "%s", "Hold / Free run measurements",
        map_values=True,
        values={True: 'T1', False: 'T0'}
    )

    trigger = Instrument.setting(
        "%s", "Trigger a measurement",
        map_values=True,
        values={True: 'T2'}
    )

    clear_stb = Instrument.setting(
        "%s", "Reset Status Byte",
        map_values=True,
        values={True: 'RS'}
    )

    single_sweep = Instrument.setting(
        "%s", "Start Single Sweep",
        map_values=True,
        values={True: 'W2'}
    )

    normal_sweep = Instrument.setting(
        "%s", "Auto Sweep",
        map_values=True,
        values={True: 'W1', False: 'W0'}
    )

    software_version = Instrument.measurement(
        "SD", "Current Software Date",
        get_process=lambda x: int(float(x[0]))
    )

    def set_frequency_start(self, x, force=False):
        if force or self.shadow_frequency_start != int(x):
            self.isTuned = False
            self.isCalibrated = False
            self.shadow_frequency_start = int(x)
            self.frequency_start = x

    frequency_start = Instrument.control(
        "FAEN", "FA%dMZEN", "Start Frequency",
        set_process=lambda x: int(x),
        get_process=lambda x: int(float(x[0]) / 1e6),
        validator=truncated_range,
        values=[FREQ_MIN, FREQ_MAX]
    )

    def set_frequency_stop(self, x, force=False):
        if force or self.shadow_frequency_stop != int(x):
            self.isTuned = False
            self.isCalibrated = False
            self.shadow_frequency_stop = int(x)
            self.frequency_stop = x

    frequency_stop = Instrument.control(
        "FBEN", "FB%dMZEN", "Start Frequency",
        set_process=lambda x: int(x),
        get_process=lambda x: int(float(x[0]) / 1e6),
        validator=truncated_range,
        values=[FREQ_MIN, FREQ_MAX]
    )

    def set_frequency_step(self, x, force=False):  # No idea how to do this differently
        if force or self.shadow_frequency_step != int(x):
            self.isTuned = False
            self.isCalibrated = False
            self.shadow_frequency_step = int(x)
            self.frequency_step = x
            self.frequency_incr = x  # keep both the sweep step and the manual incrment in sync

    frequency_step = Instrument.setting(
        "SS%dMZEN", "Frequency Step",
        set_process=lambda x: int(x),
        validator=strict_range,
        values=[1, FREQ_MAX]
    )

    frequency_incr = Instrument.setting(
        "FN%dMZEN", "Frequency Increment",
        set_process=lambda x: int(x),
        validator=strict_range,
        values=[1, FREQ_MAX]
    )

    smoothing_factor = Instrument.control(
        "AF", "F%dEN", "Smoothing Factor",
        set_process=lambda x: int(x),
        get_process=lambda x: int((x[0])),
        validator=strict_discrete_set,
        values={1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7, 256: 8, 512: 9},
        map_values=True
    )

    frequency = Instrument.control(
        "FREN", "FR%dMZEN", "Frequency",
        set_process=lambda x: int(x),
        get_process=lambda x: int(float(x[0]) / 1e6)
    )

    @property
    def frequencies(self):
        """ Returns a numpy array of frequencies in Hz that
        correspond to the current settings of the instrument.
        """
        return np.arange(
            self.shadow_frequency_start,
            self.shadow_frequency_stop + self.shadow_frequency_step,
            self.shadow_frequency_step,
            dtype=np.float64
        )

    def reset(self):
        self.write("PR")  # Preset
        self.shadow_frequency_start = 30
        self.shadow_frequency_stop = 1600
        self.shadow_frequency_step = 20
        self.isTuned = False
        self.isCalibrated = False
        self.calibrate_step_started = False

        time.sleep(1)  # give the PR some time

        self.set_srq_mask(self.DATA_READY)  # SRQ on Data Ready
        self.write("J4")  # HP 8340/8341B LO
        self.data_output_full = True
        self.hold = True
        self.smoothing_factor = 1
        self.smoothing_factor = 1  # Twice because the first time fails
        self.measurement_mode = 1

    def get_frequency_step(self):
        self.data_output_full = True
        self.set_srq_mask(self.DATA_READY)
        (cur_freq, gain, nf) = self.read_measurement()
        print("current freq: " + str(cur_freq))
        self.write('UP')
        (freq, gain, nf) = self.read_measurement()
        print("new freq: " + str(freq))
        step = freq - cur_freq
        print("step: " + str(step))
        return step

    def __init__(self, resourceName, **kwargs):
        super(HP8970B, self).__init__(
            resourceName,
            "HP 8970B Noise Figure Meter",
            includeSCPI=False,
            **kwargs
        )
        self.data_output_full = True
        self.smoothing_factor = 1
        self.shadow_frequency_start = self.frequency_start
        self.shadow_frequency_stop = self.frequency_stop
        self.shadow_frequency_step = self.get_frequency_step()
        self.adapter.connection.timeout = 10000  # 10 second timeout for measurements with long averaging
