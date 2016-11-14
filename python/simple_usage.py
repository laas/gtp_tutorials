#! /usr/bin/env python

import rospy
import actionlib

import gtp_ros_msgs.msg
import gtp_ros_msgs.srv
import move3d_ros_lib.srv
import std_srvs.srv

client = None
actions = []

def pick(agent,target_object,arm_id=-1):

    #create an empty request
    goal=gtp_ros_msgs.msg.PlanGoal()
    #set the task as being a pick action
    goal.request.taskType="pick"
    #the name of the agent performing the task is given by the 'agent' parameter
    goal.request.agents.append(gtp_ros_msgs.msg.Role(role="mainAgent",name=agent))
    #the name of the object to pick is given by the parameter
    goal.request.objects.append(gtp_ros_msgs.msg.Role(role="mainObject",name=target_object))
    #we do want GTP to compute the motion plan. If false, it will only compute intermediate positions (see doc)
    goal.request.computeMotionPlan=True
    #do not update the world state automatically, we will do it manually
    goal.request.updateBefore=False

    client.send_goal(goal)
    client.wait_for_result()

    response= client.get_result().result
    print "Pick Result:"
    print response
    actions.append(response)
    return response.success

def place(agent,target_object,target_support=None):
    goal=gtp_ros_msgs.msg.PlanGoal()
    #here the action to perform is a place. it is similar to the pick action, except it has an optional supplementary parameter
    goal.request.taskType="place"
    goal.request.agents.append(gtp_ros_msgs.msg.Role(role="mainAgent",name=agent))
    goal.request.objects.append(gtp_ros_msgs.msg.Role(role="mainObject",name=target_object))
    #the support parameter is optional, is to specify where to place the object
    if target_support:
        goal.request.objects.append(gtp_ros_msgs.msg.Role(role="support",name=target_support))

    goal.request.computeMotionPlan=True
    goal.request.updateBefore=False

    client.send_goal(goal)
    client.wait_for_result()

    response= client.get_result().result
    print "Place result:"
    print response
    actions.append(response)
    return response.success

def load(action,i):
    act=gtp_ros_msgs.msg.ActionId(action.id.taskId,action.id.alternativeId)
    rospy.wait_for_service('/gtp/publishTraj')
    try:
        pub_handle=rospy.ServiceProxy('/gtp/publishTraj',gtp_ros_msgs.srv.PublishTraj)
        res=pub_handle(act,i)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        

    

def update():
    rospy.wait_for_service('/gtp/update')
    try:
        update_handle=rospy.ServiceProxy('/gtp/update',std_srvs.srv.Trigger)
        result=update_handle()
        return result.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def save_scenario(name):
    rospy.wait_for_service('/gtp/save_scenario')
    try:
        update_handle=rospy.ServiceProxy('/gtp/save_scenario',move3d_ros_lib.srv.SaveScenario)
        result=update_handle(name)
        return result.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    rospy.init_node('gtp_simple_usage')
    client=actionlib.SimpleActionClient('/gtp_server',gtp_ros_msgs.msg.PlanAction)
    client.wait_for_server()
    #the update fetch the data from toaster and updates the gtp internal world state
    #here we skip it as we want to use only internal state
    #ok=update()
    ok=True
    #save the scenario in a format that can be read by move3d-qt-studio, for debugging maybe?
    #$ move3d-qt-studio -f `rosparam get /move3d/p3dFile` -sc /tmp/init.sce
    save_scenario("/tmp/init.sce")
    if ok:
        ok = pick("PR2_ROBOT","GREY_TAPE")
        save_scenario("/tmp/picked.sce")
    if ok:
        ok = place("PR2_ROBOT","GREY_TAPE")
        save_scenario("/tmp/placed.sce")

    if ok:
        #this will publish the computed trajectories
        for act in actions:
            for sub in act.solutionParts:
                print sub.name
                load(act,sub.id)
                #the trajectory is published by gtp on /gtp/trajectory as a trajectory_msgs/JointTrajectory
                #TODO: wait for motion completion



