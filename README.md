leap_tracker
============

LEAP Motion for ROS. 
--------------------

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
