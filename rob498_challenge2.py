#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse

# Global mode variable
MODE = "NONE"

# Get states from MAVLink
CURR_STATE = State()

def state_cb(msg):
    global CURR_STATE
    CURR_STATE = msg

# Callback handlers
def handle_launch():
    MODE = "LAUNCH"
    print('Launch Requested. Your drone should take off.')

def handle_test():
    MODE = "TEST"
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    MODE = "LAND"
    print('Land Requested. Your drone should land.')

def handle_abort():
    MODE = "ABORT"
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()


if __name__ == "__main__":

    node_name = "challenge2_node"

    rospy.init_node(node_name)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Challenge 2 Services
    srv_launch = rospy.Service(node_name + '/comm/launch', Empty, callback_launch)

    srv_test = rospy.Service(node_name + '/comm/test', Empty, callback_test)

    srv_land = rospy.Service(node_name + '/comm/land', Empty, callback_land)

    srv_abort = rospy.Service(node_name + '/comm/abort', Empty, callback_abort)

    while(not rospy.is_shutdown() ):

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not CURR_STATE.connected):
            rate.sleep()

        if (MODE == "LAUNCH"):

            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2

            # Send a few setpoints before starting
            for i in range(100):   
                if(rospy.is_shutdown()):
                    break

                local_pos_pub.publish(pose)
                rate.sleep()

            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'

            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True

            last_req = rospy.Time.now()

            while(not rospy.is_shutdown() and MODE == "LAUNCH"):
                if(CURR_STATE.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(set_mode_client.call(offb_set_mode).mode_sent == True):
                        rospy.loginfo("OFFBOARD enabled")
                    
                    last_req = rospy.Time.now()
                else:
                    if(not CURR_STATE.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                        if(arming_client.call(arm_cmd).success == True):
                            rospy.loginfo("Vehicle armed")
                    
                        last_req = rospy.Time.now()

                local_pos_pub.publish(pose)

                rate.sleep()

        elif (MODE == "TEST"):

            rospy.loginfo("Testing has commenced. Hold position.")

        elif (MODE == "LAND"):

            alt_set_mode = SetModeRequest()
            alt_set_mode.custom_mode = 'ALTCTL'

            while(not rospy.is_shutdown() and MODE == "LAND"):
                if(CURR_STATE.mode != 'ALTCTL' and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(set_mode_client.call(alt_set_mode).mode_sent == True):
                        rospy.loginfo("ALTITUDE enabled")
                    
                    last_req = rospy.Time.now()

        elif (MODE == "ABORT"):
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False

            while(not rospy.is_shutdown() and MODE == "ABORT"):
                if(CURR_STATE.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle disarmed")
            
                last_req = rospy.Time.now()

        else:
            rospy.loginfo("Error: Unrecognised MODE.")