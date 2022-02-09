#!/usr/bin/env python3

from math import pi
import time
import rospy
from std_msgs.msg import String, Float64, Int32
from edocontroller import EdoFactory, SIMULATION
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from tf.transformations import euler_from_quaternion
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
#from pose_object import GazeboModelPose


# initialize the controller wanted, here the one used along with the simulation
controller = EdoFactory().create_robot_model(SIMULATION)

class GazeboModelPose:
  model_name = ''
  model_pose = Pose()
  current_pose=Pose()

  def __init__(self, model_name):
    self.modelname = model_name
    self.model_name_rectified = model_name.replace("::", "_")

    if not self.model_name:
      raise ValueError("'model_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    self.pose_pub = rospy.Publisher("edo/counter", Pose, queue_size = 10)

  def callback(self, data):
    try:
      ind = data.name.index(self.model_name)
      self.model_pose = data.pose[ind]
      self.current_pose=data.pose
    except ValueError:
      pass

class Object_avoidancecollision(object):

  def __init__(self):
    self._pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
    self._pub_aco = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=100)
    scene=controller.scene
    robot = controller.robot
    planning_frame = controller.planning_frame
    eef_link = controller.move_group.get_end_effector_link()
    group_names = controller.group_names
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = controller.move_group
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    

    collision_pose=Pose()
    collision_pose.position=[0.8,0,0]
    collision_pose.orientation=[0,0,0,1]
    radius=2

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = 'sphere'
    #co.header = collision_pose.header
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [radius]
    co.primitives = [sphere]
    co.primitive_poses = [collision_pose]
    self._pub_co.publish()

    rospy.sleep(2)

    #    group=controller.move_group
    scene=controller.scene

    obstacle=PoseStamped()
    obstacle.header.frame_id = controller.planning_frame
    obstacle.pose.position.x = 0.8
    obstacle.pose.position.y = 1
    obstacle.pose.position.z = 1
    scene.add_box("box", obstacle, (0.4,0.4,0.6))
  
    rospy.loginfo('objects attached to base_link')

    rospy.sleep(8)
    plan=group.plan()
    
    rospy.sleep(5)



if __name__ == "__main__":
  #initialize the node
  rospy.init_node("simple joint control", anonymous=True)
   
  #gp.model_name='coke_can'
  #gp = GazeboModelPose(
  #gp.pose_pub.publish(gp.model_pose)

  def callback(data):
    global x
    global y
    global z
    global w

    target_pose=Pose()
    target_pose.position=data.position
    target_pose.orientation=data.orientation

    #set up edo settings
    controller.set_velocity(0.9)
    controller.set_target(target_pose)

    controller._go_to_moveit_pose_goal(target_pose) 
    
    time.sleep(3)
    controller.reset_position()

  sub=rospy.Subscriber('counter',Pose,callback)
  rospy.spin()

