#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty

class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
    def execute(self, userdata):
        rospy.sleep(2)
        return 'init_done'

class foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foo_succeeded', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                print "state foo is being preempted!!!"
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.2)
        return 'foo_succeeded'

def child_term_cb(outcome_map):
    if outcome_map['SPEEDUP_FH'] == 'foo_succeeded':
        return True
    elif outcome_map['CHECK_FH'] == 'invalid':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['CHECK_FH'] == 'invalid':
        return 'foo_done'
    elif outcome_map['SPEEDUP_FH'] == 'foo_succeeded':
        return 'foo_reset'
    else:
        return 'foo_done'


def child_term_cb2(outcome_map):
    if outcome_map['SPEEDUP_PF'] == 'foo_succeeded':
        return True
    elif outcome_map['CHECK_PF'] == 'invalid':
        return True
    else:
        return False

def out_cb2(outcome_map):
    if outcome_map['CHECK_PF'] == 'invalid':
        return 'foo_done'
    elif outcome_map['SPEEDUP_PF'] == 'foo_succeeded':
        return 'foo_reset'
    else:
        return 'foo_done'


def monitor_cb(ud, msg):
    return False     

def main():
    rospy.init_node("preemption_example")

    foo_concurrence = smach.Concurrence(outcomes=['foo_reset', 'foo_done'],
                                        default_outcome='foo_reset',
                                        child_termination_cb=child_term_cb,
                                        outcome_cb=out_cb)

    with foo_concurrence:
        smach.Concurrence.add('SPEEDUP_FH', foo())
        smach.Concurrence.add('CHECK_FH', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


    foo_concurrence2 = smach.Concurrence(outcomes=['foo_reset', 'foo_done'],
                                        default_outcome='foo_reset',
                                        child_termination_cb=child_term_cb2,
                                        outcome_cb=out_cb2)

    with foo_concurrence2:
        smach.Concurrence.add('SPEEDUP_PF', foo())
        smach.Concurrence.add('CHECK_PF', smach_ros.MonitorState("/sm_reset2", Empty, monitor_cb))


    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('TRY_SPEEDUP_FH', init(), transitions={'init_done':'FH'})
        smach.StateMachine.add('FH', foo_concurrence, transitions={'foo_reset':'TRY_AGAIN_FH', 'foo_done':'TRY_SPEEDUP_PF'}) 
        smach.StateMachine.add('TRY_AGAIN_FH', foo(), transitions={'foo_succeeded':'FH', 'preempted':'TRY_SPEEDUP_FH'})
	smach.StateMachine.add('TRY_SPEEDUP_PF', init(), transitions={'init_done':'PF'})
	smach.StateMachine.add('PF', foo_concurrence2, transitions={'foo_reset':'TRY_AGAIN_PF', 'foo_done':'DONE'})
	smach.StateMachine.add('TRY_AGAIN_PF', foo(), transitions={'foo_succeeded':'PF', 'preempted':'TRY_SPEEDUP_PF'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/INPUT_COMMAND')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
