"""Created on 08 apr 2016

@author: Filip Lindau
"""
import sys
import PyTango
import AD7991_control as ad
import threading
import logging
import time
import numpy as np
import Queue

logging.basicConfig(format='%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s', level=logging.INFO)


class Command:
    def __init__(self, command, data=None):
        self.command = command
        self.data = data

# ==================================================================
#   AD7991DS Class Description:
#
#         Control of a AD7991 ADC
#
# ==================================================================
#     Device States Description:
#
#   DevState.ON :       Connected to Halcyon driver
#   DevState.OFF :      Disconnected from Halcyon
#   DevState.FAULT :    Error detected
#   DevState.UNKNOWN :  Communication problem
#   DevState.MOVING :  Motor moving
#   DevState.INIT :     Initializing Halcyon driver.
# ==================================================================


class AD7991DS(PyTango.Device_4Impl):

# --------- Add your global variables here --------------------------

# ------------------------------------------------------------------
#     Device constructor
# ------------------------------------------------------------------
    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        AD7991DS.init_device(self)

# ------------------------------------------------------------------
#     Device destructor
# ------------------------------------------------------------------
    def delete_device(self):
        with self.streamLock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stopThread()


# ------------------------------------------------------------------
#     Device initialization
# ------------------------------------------------------------------
    def init_device(self):
        self.streamLock = threading.Lock()
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::init_device()")))
        self.set_state(PyTango.DevState.UNKNOWN)
        self.get_device_properties(self.get_device_class())

        # Try stopping the stateThread if it was started before. Will fail if this
        # is the initial start.
        try:
            self.stopThread()

        except Exception, e:
            pass

        self.attrLock = threading.Lock()
        self.eventIdList = []
        self.stateThread = threading.Thread()
        threading.Thread.__init__(self.stateThread, target=self.stateHandlerDispatcher)

        self.commandQueue = Queue.Queue(100)

        self.stateHandlerDict = {PyTango.DevState.ON: self.onHandler,
                            PyTango.DevState.MOVING: self.onHandler,
                            PyTango.DevState.ALARM: self.onHandler,
                            PyTango.DevState.FAULT: self.faultHandler,
                            PyTango.DevState.INIT: self.initHandler,
                            PyTango.DevState.UNKNOWN: self.unknownHandler,
                            PyTango.DevState.OFF: self.offHandler}

        self.stopStateThreadFlag = False

        self.stateThread.start()

    def stateHandlerDispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        prevState = self.get_state()
        while self.stopStateThreadFlag == False:
            try:
                self.stateHandlerDict[self.get_state()](prevState)
                prevState = self.get_state()
            except KeyError:
                self.stateHandlerDict[PyTango.DevState.UNKNOWN](prevState)
                prevState = self.get_state()

    def stopThread(self):
        """Stops the state handler thread by setting the stopStateThreadFlag
        """
        self.stopStateThreadFlag = True
        self.stateThread.join(3)

    def unknownHandler(self, prevState):
        """Handles the UNKNOWN state, before communication with the hardware devices
        has been established. Here all devices are initialized.
        """
        with self.streamLock:
            self.info_stream('Entering unknownHandler')
        connectionTimeout = 1.0
        self.set_status('Connecting to AD7991 through i2c bus')
        while self.stopStateThreadFlag is False:
            # ADC:
            try:
                with self.streamLock:
                    self.info_stream(''.join(('Opening ad7991 device on address ', str(self.i2c_address))))
                self.ad7991Device = ad.AD7991Control(self.i2c_address, self.i2c_bus)
                self.ad7991Device.write_config(self.ad7991Device.compile_config())
            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Could not connect to ad7991 on address ', str(self.i2c_address))))
                with self.streamLock:
                    self.error_stream(str(ex))
                self.set_status('Could not connect to ad7991')
                self.checkCommands(blockTime=connectionTimeout)
                continue
            self.set_state(PyTango.DevState.INIT)
            break

    def initHandler(self, prevState):
        """Handles the INIT state. Query Halcyon device to see if it is alive.
        """
        with self.streamLock:
            self.info_stream('Entering initHandler')
        waittime = 1.0
        self.set_status('Initializing device')
        retries = 0
        maxtries = 5

        while self.stopStateThreadFlag == False:
            retries += 1
            if retries > maxtries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break
            try:
                self.ad_result_raw = [0, 0, 0, 0]
                self.ad_result = [0.0, 0.0, 0.0, 0.0]

                attrs = self.get_device_attr()
                self.voltage_reference = attrs.get_w_attr_by_name('VoltageReference').get_write_value()
                s = ''.join(('Voltage reference ', str(self.voltage_reference)))
                self.debug_stream(s)
                if self.voltage_reference in ['ext', 'external']:
                    self.voltage_calib = 2.0 / 4095
                    self.use_channels = [True, True, True, False]
                else:
                    self.voltage_calib = 3.3 / 4095
                    self.use_channels = [True, True, True, True]

                self.ad7991Device.set_reference(self.voltage_reference)
                for ch, enable in enumerate(self.use_channels):
                    self.ad7991Device.set_channel_enable(ch, enable)


            except Exception, ex:
                with self.streamLock:
                    self.error_stream(''.join(('Error when initializing device')))
                    self.error_stream(str(ex))
                self.checkCommands(blockTime=waittime)
                continue

            self.set_state(PyTango.DevState.ON)
            break

    def onHandler(self, prevState):
        """Handles the ON state. Connected to the AD7991.
        Loops reading A/D and checking commands.
        """
        with self.streamLock:
            self.info_stream('Entering onHandler')
        handledstates = [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.MOVING]
        waittime = 0.025
        self.set_status('On')
        while self.stopStateThreadFlag is False:
            # self.info_stream('onhandler loop')
            with self.attrLock:
                state = self.get_state()
            if state not in handledstates:
                self.info_stream(''.join(('Exit onhandler, state=', str(state), ', stopStateThreadFlag=', str(self.stopStateThreadFlag))))
                break

            # Read ad values:
            with self.attrLock:
                try:
                    for ind, ch in enumerate(self.use_channels):
                        if ch is True:
                            data = self.ad7991Device.read_ad_result()
                            self.ad_result_raw[data[0]] = data[1]
                        else:
                            self.ad_result_raw[ind] = 0
                        # self.debug_stream('AD result ok')
                        self.ad_result[ind] = self.ad_result_raw[ind] * self.voltage_calib
                except Exception, ex:
                    with self.streamLock:
                        self.error_stream(''.join(('Error reading AD result: ', str(ex))))
                    self.set_state(PyTango.DevState.FAULT)
                    self.ad_result_raw = [None, None, None, None]
                    self.ad_result = [None, None, None, None]

            self.checkCommands(blockTime=waittime)

    def faultHandler(self, prevState):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.streamLock:
            self.info_stream('Entering faultHandler')
        handledStates = [PyTango.DevState.FAULT]
        waitTime = 0.1
        retries = 0
        maxTries = 5

        while self.stopStateThreadFlag is False:
            if self.get_state() not in handledStates:
                break
            # Test ad7991:
            try:
                with self.streamLock:
                    self.info_stream('Reading AD result...')
                result = self.ad7991Device.read_ad_result()
                with self.streamLock:
                    self.info_stream(str(result))

                if result is False:
                    # Reconnect if the reply was bad:
                    self.set_state(PyTango.DevState.UNKNOWN)
                else:
                    self.set_state(PyTango.DevState.ON)

            except Exception, e:
                self.set_state(PyTango.DevState.UNKNOWN)

            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > maxTries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.checkCommands(blockTime=waitTime)

    def offHandler(self, prevState):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.streamLock:
            self.info_stream('Entering offHandler')
        self.set_state(PyTango.DevState.ON)

    def checkCommands(self, blockTime=0):
        """Checks the commandQueue for new commands. Must be called regularly.
        If the queue is empty the method exits immediately.
        """
#         with self.streamLock:
#             self.debug_stream('Entering checkCommands')
        try:
            if blockTime == 0:
                cmd = self.commandQueue.get(block=False)
            else:
                cmd = self.commandQueue.get(block=True, timeout=blockTime)

            with self.streamLock:
                self.info_stream(''.join(('Command ', str(cmd.command), ': ', str(cmd.data))))

            if cmd.command == 'off':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.OFF)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)

            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ON)

            elif cmd.command == 'writeVoltageReference':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.ad7991Device.set_reference(cmd.data)
                self.voltage_reference = cmd.data
                if self.voltage_reference in ['ext', 'external']:
                    self.voltage_calib = 2.0 / 4095
                    use_channels = [True, True, True, False]
                else:
                    self.voltage_calib = 3.3 / 4095
                    use_channels = [True, True, True, True]
                cmd_msg = Command('writeUseChannels', use_channels)
                self.commandQueue.put(cmd_msg)

            elif cmd.command == 'writeUseChannels':
                self.use_channels = cmd.data
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    for ch, enable in enumerate(cmd.data):
                        self.ad7991Device.set_channel_enable(ch, enable)

        except Queue.Empty:
#             with self.streamLock:
#                 self.debug_stream('checkCommands: queue empty')
            pass

# ------------------------------------------------------------------
#     Always excuted hook method
# ------------------------------------------------------------------
    def always_executed_hook(self):
        pass

# ------------------------------------------------------------------
#     Channel0 attribute
# ------------------------------------------------------------------
    def read_Channel0(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading Channel0')))
        with self.attrLock:
            attr_read = self.ad_result[0]
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Channel0_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     Channel1 attribute
# ------------------------------------------------------------------
    def read_Channel1(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading Channel1')))
        with self.attrLock:
            attr_read = self.ad_result[1]
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Channel1_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     Channel2 attribute
# ------------------------------------------------------------------
    def read_Channel2(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading Channel2')))
        with self.attrLock:
            attr_read = self.ad_result[2]
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Channel2_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     Channel3 attribute
# ------------------------------------------------------------------
    def read_Channel3(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading Channel3')))
        with self.attrLock:
            attr_read = self.ad_result[3]
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def is_Channel3_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.INIT,
                                PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     VoltageReference attribute
# ------------------------------------------------------------------
    def read_VoltageReference(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading voltage reference')))
        with self.attrLock:
            attr_read = self.voltage_reference
            if attr_read == None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_VoltageReference(self, attr):
        self.info_stream(''.join(('Writing voltage reference')))
        with self.attrLock:
            data = (attr.get_write_value()).lower()
            if data in ['external', 'ext', 'internal', 'int', 'vcc']:
                self.info_stream(''.join(('Setting voltage reference to ', str(data))))
                cmd_msg = Command('writeVoltageReference', data)
                self.commandQueue.put(cmd_msg)

    def is_VoltageReference_allowed(self, req_type):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     UseChannels attribute
# ------------------------------------------------------------------
    def read_UseChannels(self, attr):
        with self.streamLock:
            self.info_stream(''.join(('Reading use channels data')))
        with self.attrLock:
            attr_read = self.use_channels
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_UseChannels(self, attr):
        self.info_stream(''.join(('Writing use channel data')))
        with self.attrLock:
            data = attr.get_write_value()
            if data.__len__() == 4:
                proceed = True
                for ch in data:
                    if ch not in [0, 1, False, True]:
                        proceed = False
                if proceed == True:
                    self.info_stream(''.join(('Setting UseChannels to ', str(data))))
                    cmd_msg = Command('writeUseChannels', data)
                    self.commandQueue.put(cmd_msg)

    def is_UseChannels_allowed(self, req_type):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


# ==================================================================
#
#     AD7991DS command methods
#
# ==================================================================

# ------------------------------------------------------------------
#     On command:
#
#     Description: Start Halcyon driver
#
# ------------------------------------------------------------------
    def On(self):
        with self.streamLock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmdMsg = Command('on')
        self.commandQueue.put(cmdMsg)

# ---- On command State Machine -----------------
    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True


# ==================================================================
#
#     AD7991DSClass class definition
#
# ==================================================================
class AD7991DSClass(PyTango.DeviceClass):

    #     Class Properties
    class_property_list = {
        }

    #     Device Properties
    device_property_list = {
        'i2c_bus':
            [PyTango.DevLong,
            "I2C bus the AD7991 is connected to",
            [ 1 ] ],
        'i2c_address':
            [PyTango.DevLong,
             "I2C address the AD7991 is using",
             [0x28]],

    }

    #     Command definitions
    cmd_list = {
        'On':
            [[PyTango.DevVoid, ""],
            [PyTango.DevVoid, ""]],
            }

    #     Attribute definitions
    attr_list = {
        'Channel0':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description': "A/D result for channel 0",
                'unit': 'V',
            }],
        'Channel1':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description': "A/D result for channel 1",
                'unit': 'V',
            }],
        'Channel2':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description': "A/D result for channel 2",
                'unit': 'V',
            }],
        'Channel3':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'description': "A/D result for channel 3",
                'unit': 'V',
            }],
        'VoltageReference':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE],
            {
                'description': "Voltage reference, vcc or external (= 2.0V)",
                'unit': '',
                'Memorized': "true",
            }],


        }

# ------------------------------------------------------------------
#     AD7991DSClass Constructor
# ------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name);
        print "In AD7991DSClass constructor"

# ==================================================================
#
#     AD7991DS class main method
#
# ==================================================================
if __name__ == '__main__':
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(AD7991DSClass, AD7991DS, 'AD7991DS')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed, e:
        print '-------> Received a DevFailed exception:', e
    except Exception, e:
        print '-------> An unforeseen exception occured....', e
