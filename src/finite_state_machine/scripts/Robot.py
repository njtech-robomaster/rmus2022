
from __future__ import print_function
from re import X
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool,Int8
from apriltag_msgs.msg import ApriltagMarkerArray



class Robot(object):
    def __init__(self):
        self.statePub = rospy.Publisher('state', String, queue_size=10)
        self.graspls = rospy.Subscriber(
            'grasp_success', Bool, self.graspCallback)
        self.planels = rospy.Subscriber(
            'plane_success', Bool, self.planeCallback)
        self.observe = rospy.Subscriber(
            '/markers', ApriltagMarkerArray, self.ObserveCallback)
        self.sinkNum=rospy.Publisher('sinkNum', String, queue_size=10)


        self.isCatch = False
        self.isPlane = False
        self.isGetNum = False
        self.isFinish = False

        self.onObserve = False

        self.aruco = [0, 0, 0]
        self.arucoNum = 0
        self.findoreCount = 0

        self.Rate = rospy.Rate(1)

    def movebase(self, x, y, z, tx, ty, tz, tw):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.loginfo("Client")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = tx
        goal.target_pose.pose.orientation.y = ty
        goal.target_pose.pose.orientation.z = tz
        goal.target_pose.pose.orientation.w = tw
        rospy.loginfo("send_goal")
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("move_base server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def move(self, x,y,z,tx,ty,tz,tw):
        result = self.movebase(x,y,z,tx,ty,tz,tw)
        try:
            while not result:
                pass
        except rospy.ROSException:
            raise TimeoutError

    def graspCallback(self, data):
        if data.data:
            self.isCatch = True
        else:
            self.isCatch = False

    def planeCallback(self, data):
        if data.data:
            self.isPlane = True
        else:
            self.isPlane = False

    def ObserveCallback(self, data):
        data = data.markers
        if not self.onObserve:
            return
        if len(data) == 1:
            rospy.logwarn("Only see one tag")
            return

        self.arucoNum = 0
        self.isGetNum = False

        
        arucoGet = []
        for x in data:
            if x.id in [0, 1, 2, 3, 4]:
                arucoGet.append(x)
        arucoGet = sorted(
            arucoGet, key=lambda x: x.pose.position.z, reverse=False)
        for x in range(0, min(3, len(arucoGet))):
            self.aruco[x] = arucoGet[x].id+1
            self.arucoNum += 1
            self.isGetNum = True

    def exitObserve(self):
        self.onObserve = False

