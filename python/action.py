#! /usr/bin/env python

import roslib; roslib.load_manifest('gtp_tutorials')
import rospy
import actionlib
import sys

import gtp_ros_msgs.msg
import gtp_ros_msgs.srv
import toaster_msgs.msg
import toaster_msgs.srv
import pr2motion.msg
import pr2motion.srv
import move3d_ros_lib.srv
import std_srvs.srv

import argparse

class PlanMgr:

    client = None
    pr2motionclient=None
    pr2gripperclient=None
    actions = []

    default_traj_mode = 1 #use Gatech trajectories, SoftMotion is buggy (returns immediately)

    def __init__(self):
        self.client=actionlib.SimpleActionClient('/gtp_server',gtp_ros_msgs.msg._PlanAction.PlanAction)
        #self.client.wait_for_server()

        self.pr2motionclient = actionlib.SimpleActionClient('/pr2motion/Arm_Move',pr2motion.msg.Arm_MoveAction)
        #self.pr2motionclient.wait_for_server()

        self.pr2gripperclient = actionlib.SimpleActionClient('/pr2motion/Gripper_Operate',pr2motion.msg.Gripper_OperateAction)
        #self.pr2gripperclient.wait_for_server()


    def plan(self,action,agent,target_agent=None,main_object=None,target_support=None,prev_act=None,arm=None,update=False,attachment=None):
        if prev_act:
            prev_act='('+prev_act+')'
            prev_act=eval(prev_act)
            if not type(prev_act) == tuple:
                print ("error")
        if attachment:
            attachment='('+attachment+')'
            attachment=eval(attachment)
            if not type(attachment) == tuple:
                print ("error")

        ok = self.client.wait_for_server()
        if not ok:
            print("could not connect to action server")
            sys.exit(1)
        goal=gtp_ros_msgs.msg.PlanGoal()
        goal.request.taskType=action
        goal.request.agents.append(gtp_ros_msgs.msg.Role(role="mainAgent",name=agent))
        if main_object:
            goal.request.objects.append(gtp_ros_msgs.msg.Role(role="mainObject",name=main_object))
        if target_support:
            goal.request.objects.append(gtp_ros_msgs.msg.Role(role="support",name=target_support))
        if target_agent:
            goal.request.agents.append(gtp_ros_msgs.msg.Role(role="targetAgent",name=target_agent))
        if prev_act:
            goal.request.previousAction.taskId=prev_act[0]
            goal.request.previousAction.taskId=prev_act[1]
        if attachment:
            goal.request.setAttachmentsFrom.append(gtp_ros_msgs.msg.ActionId(attachment[0],attachment[1]))
        else:
            goal.request.previousAction.taskId = -2
            goal.request.previousAction.alternativeId = -2

        goal.request.computeMotionPlan=True
        goal.request.updateBefore=update

        print ("sending goal")
        self.client.send_goal(goal,feedback_cb=self.onprogress)
        self.client.wait_for_result()

        response= self.client.get_result().result
        print "Plan result:"
        print response
        self.actions.append(response)
        return response.success

    def onprogress(self,evt):
        print("%s" % evt)

if __name__ == "__main__":
    rospy.init_node('gtp_simple_usage',anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('action',type=str)
    parser.add_argument('agent',type=str)
    parser.add_argument('-o','--main-object',type=str)
    parser.add_argument('-a','--target-agent',type=str)
    #parser.add_argument('-A','--arm',help='an integer for the id of the arm to use, by default use whichever',type=int)
    parser.add_argument('-p','--prev-act',metavar='M,N',type=str,default='-2,-2')
    parser.add_argument('-u','--update',action="store_true",help='update the GTP worldstate before planning')
    parser.add_argument('-g','--attachment',metavar='M,N',help='a pair of integers indicating a task id to copy the attachments from it (e.g: place -u -g 2,1). Default=None',type=str,default=None)

    args = parser.parse_args()
    print("%s" % args)

    superv=PlanMgr()
    superv.plan(**vars(args))

