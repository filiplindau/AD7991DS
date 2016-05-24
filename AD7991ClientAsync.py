import PyTango as pt
import sys
from PyQt4 import QtGui
import time


class UpdateChannel():
    def __init__(self):
        pass

    def attr_read(self, attr):
        print 'Update Channel'
        print attr
        return attr

    def attr_write(self, *arg):
        print 'Written'

class AD7991ClientAsync(QtGui.QWidget):

    def __init__(self, device_name='gunlaser/devices/ad7991-0', parent=None):
        QtGui.QWidget.__init__(self, parent)

        self.ch0_label = QtGui.QLabel('')
        self.ch0_label.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.ch1_label = QtGui.QLabel('')
        self.ch1_label.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.ch2_label = QtGui.QLabel('')
        self.ch2_label.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.ch3_label = QtGui.QLabel('')
        self.ch3_label.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)

        self.vcc_button = QtGui.QPushButton('Vcc')
        self.vcc_button.pressed.connect(self.vcc_pressed)

        self.init_button = QtGui.QPushButton('Init')
        self.init_button.pressed.connect(self.init_pressed)

        self.mainLayout = QtGui.QGridLayout()
        self.mainLayout.addWidget(QtGui.QLabel('Channel 0'), 0, 0)
        self.mainLayout.addWidget(self.ch0_label, 0, 1)
        self.mainLayout.addWidget(QtGui.QLabel('Channel 1'), 1, 0)
        self.mainLayout.addWidget(self.ch1_label, 1, 1)
        self.mainLayout.addWidget(QtGui.QLabel('Channel 2'), 2, 0)
        self.mainLayout.addWidget(self.ch2_label, 2, 1)
        self.mainLayout.addWidget(QtGui.QLabel('Channel 3'), 3, 0)
        self.mainLayout.addWidget(self.ch3_label, 3, 1)
        self.mainLayout.addWidget(QtGui.QLabel('Push for Vcc ref'), 4, 0)
        self.mainLayout.addWidget(self.vcc_button, 4, 1)
        self.mainLayout.addWidget(QtGui.QLabel('Push for Init'), 5, 0)
        self.mainLayout.addWidget(self.init_button, 5, 1)

        self.setLayout(self.mainLayout)

        self.device = pt.DeviceProxy(device_name)

        attr = self.device.read_attribute('channel0')
        print attr.value
        self.ch0_label.setText(str(attr.value))

        apiutil = pt.ApiUtil.instance()
        apiutil.set_asynch_cb_sub_model(pt.cb_sub_model.PUSH_CALLBACK)
        print apiutil.get_asynch_cb_sub_model()

        self.device.read_attribute_asynch('channel0', self.update_channel)
        self.device.read_attribute_asynch('channel1', self.update_channel)
        self.device.read_attribute_asynch('channel2', self.update_channel)
        self.device.read_attribute_asynch('channel3', self.update_channel)

        self.cb_obj = UpdateChannel()

    def update_channel(self, arg):
        name = arg.attr_names[0].lower()
        attr = arg.argout
        if name == 'channel0':
            self.ch0_label.setText(str(attr[0].value))
            self.device.read_attribute_asynch('channel0', self.update_channel)
        elif name == 'channel1':
            self.ch1_label.setText(str(attr[0].value))
            self.device.read_attribute_asynch('channel1', self.update_channel)
        elif name == 'channel2':
            self.ch2_label.setText(str(attr[0].value))
            self.device.read_attribute_asynch('channel2', self.update_channel)
        elif name == 'channel3':
            self.ch3_label.setText(str(attr[0].value))
            self.device.read_attribute_asynch('channel3', self.update_channel)
        time.sleep(0.025)

    def vcc_pressed(self):
        print 'Vcc pressed'
        # self.device.write_attribute('voltagereference', 'vcc')
        self.device.write_attribute('voltagereference', 'vcc', wait=False)

    def init_pressed(self):
        print 'Init pressed'
        # self.device.write_attribute('voltagereference', 'vcc')
        self.device.command_inout_asynch('init', self.update_init)

    def update_vcc(**kwargs):
        print 'Update vcc '
        # print arg, type(arg)

    def update_init(self, *arg):
        print 'Update init '
        print arg

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    myapp = AD7991ClientAsync()
    myapp.show()

    sys.exit(app.exec_())