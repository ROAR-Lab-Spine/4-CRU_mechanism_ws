import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from control_msgs.msg import JointJog
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
# from trajectory_msgs.msg import JointTrajectoryPoint

# Create your own subclass of the parent class Plugin
class MyPlugin(Plugin):
    """
    Widget for use with 4-CRU mechanism motor control
    Handles all widget callbacks of GUI components defined in .ui file
    """

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(MyPlugin, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
          dest="quiet",
          help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_robot_ctrl_gui'), 'resource', '4-CRU_gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Enable the sliders value to be tracked all the time
        self._widget.verticalSlider_1.setTracking(True)
        self._widget.verticalSlider_2.setTracking(True)
        self._widget.verticalSlider_3.setTracking(True)
        self._widget.verticalSlider_4.setTracking(True)

        # Connect Event Handlers (callbacks) to GUI components in 4-CRU_gui.ui
        self._widget.verticalSlider_1.valueChanged[int].connect(self._handle_slider_1_value_changed)
        self._widget.verticalSlider_2.valueChanged[int].connect(self._handle_slider_2_value_changed)
        self._widget.verticalSlider_3.valueChanged[int].connect(self._handle_slider_3_value_changed)
        self._widget.verticalSlider_4.valueChanged[int].connect(self._handle_slider_4_value_changed)

        self._widget.radioButton_1.toggled[bool].connect(self._handle_radio_1_toggled)
        self._widget.radioButton_2.toggled[bool].connect(self._handle_radio_2_toggled)

        # Set up Publishers
        self.pubJointPosCmd = TopicPublisher('joint_pos_cmd', JointJog)
        self.pubJointPosCmd._message.joint_names = ["motor_1", "motor_2", "motor_3", "motor_4"]
        self.pubJointPosCmd._message.displacements = self.getAllSliderValues()
        self.pubJointPosCmd._message.header.frame_id = "joint_pos_cmd"
        self.pubJointPosCmd._message.header.stamp = rospy.Time.now()

        init_slider_pos = self.getAllSliderValues()
        self.pubJointPosCmdMotor1 = TopicPublisher('joint_pos_cmd_motor_1', Float32)
        self.pubJointPosCmdMotor2 = TopicPublisher('joint_pos_cmd_motor_2', Float32)
        self.pubJointPosCmdMotor3 = TopicPublisher('joint_pos_cmd_motor_3', Float32)
        self.pubJointPosCmdMotor4 = TopicPublisher('joint_pos_cmd_motor_4', Float32)
        self.pubJointPosCmdMotor1._message.data = init_slider_pos[0]
        self.pubJointPosCmdMotor2._message.data = init_slider_pos[1]
        self.pubJointPosCmdMotor3._message.data = init_slider_pos[2]
        self.pubJointPosCmdMotor4._message.data = init_slider_pos[3]

        def shutdown_plugin(self):
        # TODO unregister all publishers here
            pass

        def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
            pass

        def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
            pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    # Callback function of GUI components
    def _handle_slider_1_value_changed(self, value):
        # Publish the commanded jont position values
        print "motor 1 position changed to", value, " mm"
        self.publishJointPosCmd(0, value)

    def _handle_slider_2_value_changed(self, value):
        # Publish the commanded jont position values
        print "motor 2 position changed to", value, " mm"
        self.publishJointPosCmd(1, value)

    def _handle_slider_3_value_changed(self, value):
        # Publish the commanded jont position values
        print "motor 3 position changed to", value, " mm"
        self.publishJointPosCmd(2, value)

    def _handle_slider_4_value_changed(self, value):
        # Publish the commanded jont position values
        print "motor 4 position changed to", value, " mm"
        self.publishJointPosCmd(3, value)

    def _handle_radio_1_toggled(self, checked):
        print "Enable Motors toggled to", checked

    def _handle_radio_2_toggled(self, checked):
        print "Enable PIDs toggled to", checked

    def getAllSliderValues(self):
        joint_slider_values = []
        joint_slider_values.append(self._widget.verticalSlider_1.value())
        joint_slider_values.append(self._widget.verticalSlider_2.value())
        joint_slider_values.append(self._widget.verticalSlider_3.value())
        joint_slider_values.append(self._widget.verticalSlider_4.value())
        return joint_slider_values

    def publishJointPosCmd(self, joint_index, value):
        self.pubJointPosCmd._message.displacements[joint_index] = value
        self.pubJointPosCmd._message.header.stamp = rospy.Time.now()     
        self.pubJointPosCmd.publish()

        if joint_index==0:
            self.pubJointPosCmdMotor1._message.data = value
            self.pubJointPosCmdMotor1.publish()
        elif joint_index==1:
            self.pubJointPosCmdMotor2._message.data = value
            self.pubJointPosCmdMotor2.publish()
        elif joint_index==2:
            self.pubJointPosCmdMotor3._message.data = value
            self.pubJointPosCmdMotor3.publish()
        elif joint_index==3:
            self.pubJointPosCmdMotor4._message.data = value
            self.pubJointPosCmdMotor4.publish()
        else:
            print "exceeding joint index"

class TopicPublisher(object):

    def __init__(self, topic_name, message_class):
        self._name = topic_name
        try:
            self._publisher = rospy.Publisher(
                topic_name, message_class, queue_size=100)
            self._message = message_class()
        except ValueError as e:
            rospy.logfatal('msg file for %s not found' % topic_name)
            raise e

    def get_topic_name(self):
        return self._name

    def publish(self):
        self._publisher.publish(self._message)

    def get_message(self):
        return self._message