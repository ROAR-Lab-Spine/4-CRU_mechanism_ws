import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from control_msgs.msg import JointJog
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

from kinematic_model.srv import RobotIK, RobotIKRequest, RobotIKResponse

import tf
from tf import transformations as tfs 
import math

import geometry_msgs.msg
from geometry_msgs.msg import PoseArray, Pose
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

        self._widget.verticalSlider_x.setTracking(True)
        self._widget.verticalSlider_y.setTracking(True)
        self._widget.verticalSlider_z.setTracking(True)
        self._widget.verticalSlider_pitch_x.setTracking(True)
        self._widget.verticalSlider_roll_y.setTracking(True)
        self._widget.verticalSlider_yaw_z.setTracking(True)

        # Connect Event Handlers (callbacks) to GUI components in 4-CRU_gui.ui
        self._widget.verticalSlider_1.valueChanged[int].connect(self._handle_slider_1_value_changed)
        self._widget.verticalSlider_2.valueChanged[int].connect(self._handle_slider_2_value_changed)
        self._widget.verticalSlider_3.valueChanged[int].connect(self._handle_slider_3_value_changed)
        self._widget.verticalSlider_4.valueChanged[int].connect(self._handle_slider_4_value_changed)

        self._widget.verticalSlider_x.valueChanged[int].connect(self._handle_slider_pose_value_changed)
        self._widget.verticalSlider_y.valueChanged[int].connect(self._handle_slider_pose_value_changed)
        self._widget.verticalSlider_z.valueChanged[int].connect(self._handle_slider_pose_value_changed)
        self._widget.verticalSlider_pitch_x.valueChanged[int].connect(self._handle_slider_pose_value_changed)
        self._widget.verticalSlider_roll_y.valueChanged[int].connect(self._handle_slider_pose_value_changed)
        self._widget.verticalSlider_yaw_z.valueChanged[int].connect(self._handle_slider_pose_value_changed)

        # self._widget.radioButton_1.toggled[bool].connect(self._handle_radio_1_toggled)


        self._widget.pushButton_1.clicked[bool].connect(self._handle_push_1_clicked)
        self._widget.pushButton_2.clicked[bool].connect(self._handle_push_2_clicked)
        self._widget.pushButton_2.clicked[bool].connect(self._handle_push_2_clicked)
        self._widget.pushButtonExecuteTraj.clicked[bool].connect(self._handle_push_execute_traj_clicked)

        # Set up Publishers
        self.pubJointPosCmd = TopicPublisher('joint_pos_cmd', JointJog)
        self.pubJointPosCmd._message.joint_names = ["motor_1", "motor_2", "motor_3", "motor_4"]
        self.pubJointPosCmd._message.displacements = self.getAllJointSliderValues()
        self.pubJointPosCmd._message.header.frame_id = "joint_pos_cmd"
        self.pubJointPosCmd._message.header.stamp = rospy.Time.now()

        init_slider_pos = self.getAllJointSliderValues()
        self.pubJointPosCmdMotor1 = TopicPublisher('joint_pos_cmd_motor_1', Float32)
        self.pubJointPosCmdMotor2 = TopicPublisher('joint_pos_cmd_motor_2', Float32)
        self.pubJointPosCmdMotor3 = TopicPublisher('joint_pos_cmd_motor_3', Float32)
        self.pubJointPosCmdMotor4 = TopicPublisher('joint_pos_cmd_motor_4', Float32)
        self.pubJointPosCmdMotor1._message.data = init_slider_pos[0]
        self.pubJointPosCmdMotor2._message.data = init_slider_pos[1]
        self.pubJointPosCmdMotor3._message.data = init_slider_pos[2]
        self.pubJointPosCmdMotor4._message.data = init_slider_pos[3]

        # Setup service proxy for IK provided by robot_4cru node
        rospy.wait_for_service('robot_4cru')
        self.robot_ik_service = rospy.ServiceProxy('robot_4cru', RobotIK)
        self.ik_req = RobotIKRequest()
        self.update_ik_req(self.getAllEndEffectorPoseSliderValues())
        self.execute_traj()

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
    # TODO: shrink to fewer handler functions
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

    # def _handle_radio_1_toggled(self, checked):
    #     print "Enable Motors toggled to", checked

    def _handle_push_1_clicked(self, checked):
        self._widget.verticalSlider_1.setSliderPosition(100)
        self._widget.verticalSlider_2.setSliderPosition(100)
        self._widget.verticalSlider_3.setSliderPosition(100)
        self._widget.verticalSlider_4.setSliderPosition(100)
        print "Reset Joint Position Sliders"

    def _handle_push_2_clicked(self, checked):
        print "Reset End-Effector Pose Sliders"
        # # Disable all tracking to prevented repeated activation
        # self._widget.verticalSlider_x.setTracking(False)
        # self._widget.verticalSlider_y.setTracking(False)
        # self._widget.verticalSlider_z.setTracking(False)
        # self._widget.verticalSlider_pitch_x.setTracking(False)
        # self._widget.verticalSlider_roll_y.setTracking(False)
        # self._widget.verticalSlider_yaw_z.setTracking(False)

        self._widget.verticalSlider_x.setSliderPosition(0)
        self._widget.verticalSlider_y.setSliderPosition(0)
        self._widget.verticalSlider_z.setSliderPosition(120)
        self._widget.verticalSlider_pitch_x.setSliderPosition(0)
        self._widget.verticalSlider_roll_y.setSliderPosition(0)
        self._widget.verticalSlider_yaw_z.setSliderPosition(0)

        # Reactivate the tracking
        # self._widget.verticalSlider_x.setTracking(True)
        # self._widget.verticalSlider_y.setTracking(True)
        # self._widget.verticalSlider_z.setTracking(True)
        # self._widget.verticalSlider_pitch_x.setTracking(True)
        # self._widget.verticalSlider_roll_y.setTracking(True)
        # self._widget.verticalSlider_yaw_z.setTracking(True)

        # Trigger action once 
        # self._handle_slider_pose_value_changed(0)
    def _handle_push_execute_traj_clicked(self, checked):
        # self.update_ik_req(self.getAllEndEffectorPoseSliderValues())
        self.execute_traj()

    # End-effector pose command
    def _handle_slider_pose_value_changed(self, value):
        # Publish the commanded jont position values
        print "x position changed to", self._widget.verticalSlider_x.sliderPosition(), " mm"
        print "y position changed to", self._widget.verticalSlider_y.sliderPosition(), " mm"
        print "z position changed to", self._widget.verticalSlider_z.sliderPosition(), " mm"
        print "pitch_x position changed to", self._widget.verticalSlider_pitch_x.sliderPosition(), " deg"
        print "roll_y position changed to", self._widget.verticalSlider_roll_y.sliderPosition(), " deg"
        print "yaw_z position changed to", self._widget.verticalSlider_yaw_z.sliderPosition(), " deg \n"
        self.update_ik_req(self.getAllEndEffectorPoseSliderValues())

    def getAllJointSliderValues(self):
        joint_slider_values = []
        joint_slider_values.append(self._widget.verticalSlider_1.value())
        joint_slider_values.append(self._widget.verticalSlider_2.value())
        joint_slider_values.append(self._widget.verticalSlider_3.value())
        joint_slider_values.append(self._widget.verticalSlider_4.value())
        return joint_slider_values

    def getAllEndEffectorPoseSliderValues(self):
        eeff_pose_slider_values = []
        eeff_pose_slider_values.append(self._widget.verticalSlider_x.value())
        eeff_pose_slider_values.append(self._widget.verticalSlider_y.value())
        eeff_pose_slider_values.append(self._widget.verticalSlider_z.value())
        eeff_pose_slider_values.append(self._widget.verticalSlider_pitch_x.value())
        eeff_pose_slider_values.append(self._widget.verticalSlider_roll_y.value())
        eeff_pose_slider_values.append(self._widget.verticalSlider_yaw_z.value())
        print eeff_pose_slider_values
        return eeff_pose_slider_values

    def publishAllJointPosCmd(self, values):
        self.pubJointPosCmdMotor1._message.data = values[0]
        self.pubJointPosCmdMotor2._message.data = values[1]
        self.pubJointPosCmdMotor3._message.data = values[2]
        self.pubJointPosCmdMotor4._message.data = values[3]

        self.pubJointPosCmdMotor1.publish()
        self.pubJointPosCmdMotor2.publish()
        self.pubJointPosCmdMotor3.publish()
        self.pubJointPosCmdMotor4.publish()
        print "published joint cmd values:", values

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

    def update_ik_req(self, eeff_pose_slider_values):
        # reset value everytime for the time being (single point command only)
        self.ik_req = RobotIKRequest()
        des_pose = Pose()
        des_pose.position.x = eeff_pose_slider_values[0]
        des_pose.position.y = eeff_pose_slider_values[1]
        des_pose.position.z = eeff_pose_slider_values[2]

        quat = tfs.quaternion_from_euler(eeff_pose_slider_values[3]/180.0*math.pi, \
            eeff_pose_slider_values[4]/180.0*math.pi, eeff_pose_slider_values[5]/180.0*math.pi, axes='sxyz')
        des_pose.orientation.x = quat[0]
        des_pose.orientation.y = quat[1]
        des_pose.orientation.z = quat[2]
        des_pose.orientation.w = quat[3]
        self.ik_req.des_poses.poses.append(des_pose)
        self.ik_resp = self.robot_ik_service(self.ik_req)
        print self.ik_req, "\n"

    def execute_traj(self):
        for i in range(len(self.ik_resp.des_joint_positions.points)):
            self.publishAllJointPosCmd(self.ik_resp.des_joint_positions.points[i].positions)

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