#!/usr/bin/python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, Vector3

px = 0.0
py = 0.0
pz = 0.0

def poseCallback(gazeboModelMsg):
	rospy.loginfo("Model Names are: %s", gazeboModelMsg.name) 

def listener():

		rospy.init_node("listener")
		rospy.Subscriber("/gazebo/model_states", ModelStates, poseCallback)
		rospy.spin()

		#pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
		#rospy.init_node("talker", anonymous=True)
	
def getPositions(model, rel):
	rospy.wait_for_service("/gazebo/get_model_state")
	try:
		modelState = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		resp = modelState(model, rel)
		return resp
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)

def sendNewPositions(modelState):
	rospy.wait_for_service("/gazebo/set_model_state")
	try:
		sendPos = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		resp = sendPos(modelState)
		return resp.success
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)

if __name__ == '__main__':
	print("Requesting for model name: pioneer2dx")		
	modelState = getPositions("pioneer2dx", "")

	print("Pose is: %s", modelState.pose.position)
	x = modelState.pose.position.x
	y = modelState.pose.position.y
	z = modelState.pose.position.z

	print("Position of Pioneer2dx is: %s %s %s", str(x), str(x), str(x))
	
	# Construct the quaternion to be sent to the pioneer2dx:
	quat = Quaternion(0.0, 0.0, 0.0, 1.0)
	targetPoint = Point(20, 150, 10)

	p = Pose(targetPoint, quat)
	t = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

	modelSt = ModelState("pioneer2dx", p, t, "")

	sendNewPositions(modelSt)
	
						
