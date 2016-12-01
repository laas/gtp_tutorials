#! /usr/bin/env python

import rospy
import actionlib

import gtp_ros_msgs.msg
import gtp_ros_msgs.srv
import toaster_msgs.msg
import toaster_msgs.srv
import pr2motion.msg
import pr2motion.srv
import move3d_ros_lib.srv
import std_srvs.srv

client = None
pr2motionclient=None
pr2gripperclient=None
actions = []

default_traj_mode = 1 #use Gatech trajectories, SoftMotion is buggy (returns immediately)

def pick(agent,target_object,prev_act=None,arm_id=-1):

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
    if arm_id == 0:
        goal.request.data.append(gtp_ros_msgs.msg.MiscData("hand","right"))
    elif arm_id == 1:
        goal.request.data.append(gtp_ros_msgs.msg.MiscData("hand","left"))
    if prev_act:
        goal.request.previousAction=prev_act.id
    else:
        goal.request.previousAction.taskId = -2
        goal.request.previousAction.alternativeId = -2



    client.send_goal(goal)
    client.wait_for_result()

    response= client.get_result().result
    print "Pick Result:"
    print response
    actions.append(response)
    return response.success

def place(agent,target_object,target_support=None,prev_act=None):
    goal=gtp_ros_msgs.msg.PlanGoal()
    #here the action to perform is a place. it is similar to the pick action, except it has an optional supplementary parameter
    goal.request.taskType="place"
    goal.request.agents.append(gtp_ros_msgs.msg.Role(role="mainAgent",name=agent))
    goal.request.objects.append(gtp_ros_msgs.msg.Role(role="mainObject",name=target_object))
    #the support parameter is optional, is to specify where to place the object
    if target_support:
        goal.request.objects.append(gtp_ros_msgs.msg.Role(role="support",name=target_support))
    if prev_act:
        goal.request.previousAction=prev_act.id
    else:
        goal.request.previousAction.taskId = -2
        goal.request.previousAction.alternativeId = -2

    goal.request.computeMotionPlan=True
    goal.request.updateBefore=False

    client.send_goal(goal)
    client.wait_for_result()

    response= client.get_result().result
    print "Place result:"
    print response
    actions.append(response)
    return response.success

def pdg_put_in_hand(action,part,obj):
    rospy.wait_for_service('/pdg/put_in_hand')
    try:
        put_handle=rospy.ServiceProxy('/pdg/put_in_hand',toaster_msgs.srv.PutInHand)
        put_handle(obj,part.agent,'pr2r_gripper_joint') # try r_gripper_palm_link
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    

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

def execute_motion(action,i):

    #gripper action?
    gripper_action=False
    gripper_mode=0
    traj_mode=default_traj_mode
    side = 1-action.solutionParts[i].armId
    if action.solutionParts[i].name == 'grasp' :
        gripper_action=True
        gripper_mode=1 #close
    if action.solutionParts[i].name == 'release' :
        gripper_action=True
        gripper_mode=0 #open
    if action.solutionParts[i].name == 'escape' or action.solutionParts[i].name == 'engage' :
        print 'special move: '+action.solutionParts[i].name
        traj_mode = 2 # path mode

    if gripper_action :
        grip_goal=pr2motion.msg.Gripper_OperateGoal()
        grip_goal.side.value=side
        grip_goal.goal_mode.value=gripper_mode
        pr2gripperclient.send_goal(grip_goal)
        rospy.sleep(2.)
        pr2gripperclient.wait_for_result()
        result=pr2gripperclient.get_result()
        print result
        return result.genom_success

    else:
        #rosaction call /pr2motion/Arm_Move '{traj_mode: {value: 0}, path_mode: {value: 0}, side: {value: 1}}'
        goal=pr2motion.msg.Arm_MoveGoal()
        goal.traj_mode.value=traj_mode
        goal.side.value = side
        goal.path_mode.value=0
        pr2motionclient.send_goal(goal)
        if traj_mode != default_traj_mode:
            rospy.sleep(5.)
        pr2motionclient.wait_for_result()
        
        result = pr2motionclient.get_result()
        print result
        return result.genom_success


if __name__ == "__main__":
    rospy.init_node('gtp_simple_usage')
    client=actionlib.SimpleActionClient('/gtp_server',gtp_ros_msgs.msg.PlanAction)
    client.wait_for_server()

    pr2motionclient = actionlib.SimpleActionClient('/pr2motion/Arm_Move',pr2motion.msg.Arm_MoveAction)
    pr2motionclient.wait_for_server()

    pr2gripperclient = actionlib.SimpleActionClient('/pr2motion/Gripper_Operate',pr2motion.msg.Gripper_OperateAction)
    pr2gripperclient.wait_for_server()

    #the update fetch the data from toaster and updates the gtp internal world state
    #here we skip it as we want to use only internal state
    #ok=update()
    ok=True
    #save the scenario in a format that can be read by move3d-qt-studio, for debugging maybe?
    #$ move3d-qt-studio -f `rosparam get /move3d/p3dFile` -sc /tmp/init.sce
    save_scenario("/tmp/init.sce")
    if ok:
        ok = pick("PR2_ROBOT","GREY_TAPE",arm_id=0)
        save_scenario("/tmp/picked.sce")
    if ok:
        ok = place("PR2_ROBOT","GREY_TAPE",prev_act = actions[-1]) # pass the pick action as previous action
        save_scenario("/tmp/placed.sce")

    if ok:
        #this will publish the computed trajectories
        for act in actions:
            for sub in act.solutionParts:
                print sub.name
                load(act,sub.id)
                ok= execute_motion(act,sub.id)
                if not ok:
                    #exit()
                    pass
                #the trajectory is published by gtp on /gtp/trajectory as a trajectory_msgs/JointTrajectory
                #TODO: wait for motion completion



