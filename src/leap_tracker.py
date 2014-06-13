#!/usr/bin/env python
# coding:utf8

"""
@package leap_tracker
@file leap_tracker.py

@brief LEAP Motion for ROS. 

This package provides a tracking server for a LEAP Motion device. 
It constantly listens to the controller for new frames and processes 
the hands and fingers tracking data. 

It publishes ROS's own JointState, TwistStamped and PoseStamped messages 
with the values of the hand's position and orientation and the fingers' 
joints angular values, and sends them all through the topics 
"leap_tracker/joint_state_out", "leap_tracker/pose_stamped_out" 
and "leap_tracker/twist_stamped_out" for whichever translation service 
listening to those topics to convert them and adapt them to any 
robot model.

@author: Óscar Gómez <oscar.gomez@uji.es>
@date 14/05/2014
""" 

def fix_import_path():
    """
    Fixes libraries path to properly import the LEAP Motion controller and 
    its Python wrapper
    """ 
    import sys, os, struct
    bit_size = struct.calcsize("P") * 8
    ARCH = '/x86' if bit_size == 32 else '/x64'
    LEAP_PATH = os.path.dirname(__file__) + '/leap'
    sys.path.extend([LEAP_PATH, LEAP_PATH + ARCH])

# Fix import path to properly import Leap controller and wrapper
fix_import_path()

import Leap, rospy, math
from exc import QuitMessageException
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf import transformations

# Initialize consts and vars
NODE_NAME = 'leap_tracker'
FRAME_ID = NODE_NAME
JS_TOPIC = '%s/joint_state_out' % NODE_NAME
PS_TOPIC = '%s/pose_stamped_out' % NODE_NAME
TS_TOPIC = '%s/twist_stamped_out' % NODE_NAME

FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']
FINGER_BONES = ['meta', 'prox', 'mid', 'dist']
POS_ATTRIBUTES = ['x', 'y', 'z']
ORI_ATTRIBUTES = ['roll', 'pitch', 'yaw']

class Logger:
    """
    @brief Wrapper for ROS logging class. 
    
    Adds color to the output.
    """
    def v(self, s, ns):
        rospy.loginfo(self.build(s, ns))
    def d(self, s, ns):
        rospy.logdebug(self.build(s, ns))
    def e(self, s, ns):
        rospy.logerr(self.build(s, ns))
    def c(self, s, ns):
        rospy.logwarn(self.build(s, ns))
    def build(self, s, ns):
        return "\x1B[1m[{}]\x1B[0m {}".format(ns, s)

LOG = Logger()

class LeapServer(Leap.Listener):
    """
    @brief Main class to get data from the LEAP Motion controller. 
    
    It extends the Leap.Listener class and implements all 
    the event methods defined in it. For more info, check the LEAP Motion API: 
    
    https://developer.leapmotion.com/documentation/skeletal/python/index.html
    """
    def on_init(self, controller):
        LOG.v("Initialized", "on_init")
        self.t = 0  # time var for automated testing
        
        # Initialize empty frame
        self.frame = None
        
        # Initialize fingers and hands
        self.hand = Leap.Hand()
        self.fingers = { FINGER_NAMES[i] : Leap.Finger() 
            for i in range(5)}
        
        # Initialize joint names for JointState messages
        self.joint_names = []
        
        # Initialize node
        rospy.init_node('hand_tracker', anonymous=True)
        
        # Initialize publishers
        self.js_pub = rospy.Publisher(JS_TOPIC, JointState, queue_size=10)
        self.ps_pub = rospy.Publisher(PS_TOPIC, PoseStamped, queue_size=10)
        self.ts_pub = rospy.Publisher(TS_TOPIC, TwistStamped, queue_size=10)
        
    def on_connect(self, controller):
        LOG.v("Connected", "on_connect")
        
    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        LOG.v("Disconnected", "on_disconnect")

    def on_exit(self, controller):
        LOG.v("END", "on_exit")

    def on_frame(self, controller):
        # Get the most recent frame and fill data structures
        frame = controller.frame()
        selected_finger = None
        
        if not frame.hands.is_empty:
            # Get the first hand
            hand = frame.hands[0]
            self.hand = hand

            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.is_empty:
                
                # Iterate fingers from leftmost to rightmost
                for i, finger in enumerate(sorted(fingers, key=lambda f: f.type())):
                    
                    # Identify thumb and pinky
                    if finger.type() == Leap.Finger.TYPE_THUMB:
                        selected_finger = FINGER_NAMES[0]
                    elif finger.type() == Leap.Finger.TYPE_PINKY:
                        selected_finger = FINGER_NAMES[-1]
                    else:
                        selected_finger = FINGER_NAMES[finger.type()]
                    
                    # Set selected finger's properties
                    self.fingers[selected_finger] = finger
                               
        # Show data through stdout 
        self.show_data(['hand'])
        
    def show_data(self, what=['hand'] + FINGER_NAMES):
        """
        @brief Shows tracking data on the standard output via 
        the logging system.
        """
        if 'hand' in what:
            normal = self.hand.palm_normal
            direction = self.hand.direction
            position = self.hand.palm_position
            LOG.v(("hand:\n" + 
                    "\tpitch: {:>6.2f} | x: {:>6.2f}\n" + \
                    "\t  yaw: {:>6.2f} | y: {:>6.2f}\n" + \
                    "\t roll: {:>6.2f} | z: {:>6.2f}")\
                .format(direction.pitch, position.x,
                        direction.yaw, position.y,
                        normal.roll, position.z), "show_data")
        for name in FINGER_NAMES:
            if name in what:
                finger = self.fingers[name]
                for b, bone_name in enumerate(FINGER_BONES):
                    bone = finger.bone(b)
                    direction = bone.direction
                    LOG.v(("{}.{}:\n" + 
                        "\tpitch: {:>6.2f}")\
                          .format(name, bone_name, direction.pitch), "show_data")
        
    def start_transmit(self):
        """
        @brief Starts transmission of tracking data.
        
        Starts sending the current tracking values via ROS topic 
        'leap_tracker' to the LEAP tracking conversion services
        """

        # Set publishing rate
        self.r = rospy.Rate(50)  # 50Hz
            
        quitting = False
        while not rospy.is_shutdown() and not quitting:
            try:
                # JointState message to publish joint positions
                js_msg = self.build_joint_state_msg()
                
                # PoseStamped messages to publish position and 
                # orientation of each joint
                ps_msg = self.build_pose_stamped_msg(test=True)
                
                # TODO: TwistStamped messages to publish linear and
                # angular velocities of each joint
                ts_msg = TwistStamped()

                # Publish the messages
                self.js_pub.publish(js_msg)
                self.ps_pub.publish(ps_msg)

                # TODO: Publish TwistStamped
                # self.ts_pub.publish(ts_msg)
                self.r.sleep()
                self.t += 0.01  # automated tests time var
                
            except KeyboardInterrupt:
                LOG.e("KeyboardInterrupt detected", "start_transmit")
                quitting = True

        LOG.d("Quit command sent to client", "main")
        raise QuitMessageException("Quit message received from client")

    def build_joint_state_msg(self):
        """
        @brief JointState message builder.
        
        Builds a JointState message with the current position of the finger 
        joints and its names.
        """
        
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        
        if self.joint_names == []:
            self.joint_names = ["{}.{}".format('hand', attr) 
                    for attr in ORI_ATTRIBUTES] + \
            ["{}.{}.{}".format(finger, bone, ori) 
                for finger in FINGER_NAMES 
                    for bone in FINGER_BONES
                        for ori in ORI_ATTRIBUTES]
            LOG.v("Publishing JointState for the following joints: {}".format(self.joint_names), "start_transmit")
        
        js_msg.position = [0.0] * len(self.joint_names)

        pos = 0
        # Build JointState. First the hand...        
        for i, attr in enumerate(ORI_ATTRIBUTES):
            js_msg.name.append('hand.' + str(attr))
            
            # Roll precision hack
            if attr == 'roll':
                vector = self.hand.palm_normal
            else:
                vector = self.hand.direction
                
            js_msg.position[pos] = getattr(vector, attr)
            pos += 1

        # ...then the fingers
        for i, finger_name, finger in \
            [(i, finger_name, self.fingers[finger_name]) \
                for i, finger_name in enumerate(FINGER_NAMES)]:
                        
            # LEAP API v2.0: Skeletal model
            # Get bones
            for j, bone_name, bone in \
                [(j, bone_name, finger.bone(j)) \
                    for j, bone_name in enumerate(FINGER_BONES)]:

                # Fill the joint values one by one
                for k, attr in enumerate(ORI_ATTRIBUTES):

                    joint_name = "{}.{}.{}".format(finger_name, bone_name, attr)
                    joint_value = getattr(bone.direction, attr)
                    
                    js_msg.name.append(joint_name)
                    js_msg.position[pos] = joint_value
                    pos += 1
                    
        # return the JointState message
        return js_msg
    
    def build_pose_stamped_msg(self, test=False):
        """
        @brief PoseStamped builder
        
        Builds a PoseStamped message with the current position of the hand 
        and its pose.
        """
    
        # Hand first
        ps_msg = PoseStamped()
        ps_msg.header.stamp = rospy.Time.now()
        ps_msg.header.frame_id = FRAME_ID
        
        # Convert to Quaternions
        direction = self.hand.direction
        position = self.hand.palm_position
        normal = self.hand.palm_normal
        
        roll = normal.roll
        pitch = normal.pitch
        yaw = direction.yaw
        
        quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
        
        # Set orientation quaternion in the message
        # type(pose) = geometry_msgs.msg.Pose
        ps_msg.pose.orientation.x = quaternion[0]
        ps_msg.pose.orientation.y = quaternion[1]
        ps_msg.pose.orientation.z = quaternion[2]
        ps_msg.pose.orientation.w = quaternion[3]
            
        if not test:
            # Set position values in the message
            for j, attr in enumerate(POS_ATTRIBUTES):
                val = getattr(position, attr)
                setattr(ps_msg.pose.position, attr, val)
        else:
            ((x, y, z), (pitch, yaw, roll)) = self.test_pose()
            ps_msg.pose.position.x = x
            ps_msg.pose.position.y = y
            ps_msg.pose.position.z = z
            
            quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
            ps_msg.pose.orientation.x = quaternion[0]
            ps_msg.pose.orientation.y = quaternion[1]
            ps_msg.pose.orientation.z = quaternion[2]
            ps_msg.pose.orientation.w = quaternion[3]
        
        # return the PoseStamped messages
        print ps_msg
        return ps_msg
    
    def test_pose(self):
        """
        @brief Generates test values for the pose messages.
        """
        t = self.t
        
        # Cyclic functions for orientation and position values
        delta = math.sin(t) * 1000
        alpha = math.cos(t) * math.pi * 2
        
        # Default values
        x = 0
        y = 0
        z = 0

        pitch = 0
        yaw = 0
        roll = 0
        
        # assign values cyclically
        if t % (math.pi * 12) < math.pi * 2:
            x = delta
        elif t % (math.pi * 12) < math.pi * 4:
            y = delta
        elif t % (math.pi * 12) < math.pi * 6:
            z = delta
        elif t % (math.pi * 12) < math.pi * 8:
            pitch = alpha
        elif t % (math.pi * 12) < math.pi * 10:
            yaw = alpha
        elif t % (math.pi * 12) < math.pi * 12:
            roll = alpha
        else:
            # Reset counter
            self.t = 0.0
            
        return ((x, y, z), (pitch, yaw, roll))
    
def main():
    # Init the server and controller
    leap_server = LeapServer()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(leap_server)
    
    # Keep this process running until quit from client or Ctrl^C
    LOG.v("Press ^C to quit...", "main")
    try:
        # Start communication
        leap_server.start_transmit()
    except QuitMessageException as e:
        LOG.e(e, "main")
    except KeyboardInterrupt as e:
        LOG.e("Interrupted by user", "main")

    # Remove the sample listener when done
    controller.remove_listener(leap_server)
      
if __name__ == '__main__':
    main()
