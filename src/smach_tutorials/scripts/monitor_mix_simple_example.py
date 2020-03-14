#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty

# define state INIT
# monitor_cb
def monitor_cb(ud, msg):
    return False

# define state Speedup_FH
class Speedup_FH(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Speedup FH...')
	rospy.sleep(1)
        return 'outcome1'

# define state Speedup_PF
class Speedup_PF(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Speedup PF...')
	rospy.sleep(1)
        return 'outcome2'

# define state Fini
class Fini(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Finish speedup process.')
        return 'outcome3'

# main
def main():
    rospy.init_node('monitor_example')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('SU_FH', Speedup_FH(), 
                               transitions={'outcome1':'CK_FH'})
	smach.StateMachine.add('CK_FH', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb), transitions={'invalid':'SU_PF', 'valid':'SU_FH', 'preempted':'SU_FH'})
	smach.StateMachine.add('SU_PF', Speedup_PF(), 
                               transitions={'outcome2':'CK_PF'})
	smach.StateMachine.add('CK_PF', smach_ros.MonitorState("/sm_reset2", Empty, monitor_cb), transitions={'invalid':'FINI', 'valid':'SU_PF', 'preempted':'SU_PF'})

        smach.StateMachine.add('FINI', Fini(), 
                               transitions={'outcome3':'outcome4'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
