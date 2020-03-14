#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty

class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
    def execute(self, userdata):
        rospy.sleep(3.5)
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
            rospy.sleep(1.0)
        return 'foo_succeeded'

def child_term_cb(outcome_map):
    if outcome_map['FOO_CALC'] == 'foo_succeeded':
        return True
    elif outcome_map['FOO_RESET'] == 'invalid':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['FOO_RESET'] == 'invalid':
        return 'foo_reset'
    elif outcome_map['FOO_CALC'] == 'foo_succeeded':
        return 'foo_done'
    else:
        return 'foo_reset'


def child_term_cb2(outcome_map):
    if outcome_map['FOO_CALC2'] == 'foo_succeeded':
        return True
    elif outcome_map['FOO_RESET2'] == 'invalid':
        return True
    else:
        return False

def out_cb2(outcome_map):
    if outcome_map['FOO_RESET2'] == 'invalid':
        return 'foo_reset'
    elif outcome_map['FOO_CALC2'] == 'foo_succeeded':
        return 'foo_done'
    else:
        return 'foo_reset'


def monitor_cb(ud, msg):
    return False     

def main():
    rospy.init_node("preemption_example")

    foo_concurrence = smach.Concurrence(outcomes=['foo_done', 'foo_reset'],
                                        default_outcome='foo_done',
                                        child_termination_cb=child_term_cb,
                                        outcome_cb=out_cb)

    with foo_concurrence:
        smach.Concurrence.add('FOO_CALC', foo())
        smach.Concurrence.add('FOO_RESET', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


    foo_concurrence2 = smach.Concurrence(outcomes=['foo_done', 'foo_reset'],
                                        default_outcome='foo_done',
                                        child_termination_cb=child_term_cb2,
                                        outcome_cb=out_cb2)

    with foo_concurrence2:
        smach.Concurrence.add('FOO_CALC2', foo())
        smach.Concurrence.add('FOO_RESET2', smach_ros.MonitorState("/sm_reset2", Empty, monitor_cb))


    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('INIT', init(), transitions={'init_done':'FOO'})
        smach.StateMachine.add('FOO', foo_concurrence, transitions={'foo_done':'BAR', 'foo_reset':'INIT2'}) 
        smach.StateMachine.add('BAR', foo(), transitions={'foo_succeeded':'FOO', 'preempted':'INIT'})
	smach.StateMachine.add('INIT2', init(), transitions={'init_done':'FOO2'})
	smach.StateMachine.add('FOO2', foo_concurrence2, transitions={'foo_done':'BAR2', 'foo_reset':'DONE'})
	smach.StateMachine.add('BAR2', foo(), transitions={'foo_succeeded':'FOO2', 'preempted':'INIT2'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
