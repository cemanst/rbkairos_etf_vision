#!/usr/bin/env python3
import rospy
from rbkairos_etf_services.srv import ActionServer, ActionServerRequest

def run_my_action(coordinates):
    
    rospy.wait_for_service('/robot/action_server')
    try:
        service_call = rospy.ServiceProxy('/robot/action_server', ActionServer)
        
        # Format: [x, y, z, R, P, Y]

        # 10.3cm to z axis to accommodate for EE->link8:
        my_coords = [coordinates[0],coordinates[1],coordinates[2]+0.103, 3.14, 0.0, 3.14-0.785] 
        rospy.loginfo("starting grasp action")
        req = ActionServerRequest()
        req.action_id = "grasp_fruit" 
        req.input = my_coords
        req.timeout = 10.0
        
        rospy.loginfo(f"{req.action_id} : {my_coords}")
        resp = service_call(req)        
        rospy.loginfo("Grasp successful, loading to bin...")
        req_bin = ActionServerRequest()
        req_bin.action_id = "load_to_bin"
        req_bin.input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        req_bin.timeout = 20.0
        
        resp2 = service_call(req_bin)
        rospy.loginfo("Loading to bin successful")
        return resp2

    except rospy.ServiceException as e:
        rospy.logerr(f"action call failed: {e}")

if __name__ == "__main__":
    run_my_action()