#!/usr/bin/env python3

import rospy
import time
import math
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, Grasp, GripperTranslation, PlaceLocation
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from edocontroller import EdoFactory, SIMULATION

controller = EdoFactory().create_robot_model(SIMULATION)
scene=controller.scene

#First method
def obstacleAdd(objPose, shape):
    patternBlock = SolidPrimitive()
    patternBlock.type = SolidPrimitive.BOX
    #shape.dimensions.resize(3)
    patternBlock.dimensions=list((0.8,0.8,0.8))
    
    cube = PoseStamped()
    cube.pose.position.x=-0.35
    cube.pose.position.y=0.0
    cube.pose.position.z=0.28+0.26
    cube.pose.orientation.x=0
    cube.pose.orientation.y=0
    cube.pose.orientation.z=-0.7
    #cube.pose.orientation.w=math.sqrt(0.5)
    size=(0.08, 0.08, 0.3)
    objColl = CollisionObject()
    objColl.header=objPose.header
    objColl.primitives = [shape]
    objColl.primitive_poses=[objPose.pose]
    objColl.header.frame_id= controller.planning_frame
    objColl.header.stamp = rospy.Time.now()
    #objColl.id = 'cafe_table'

    # Attache it to the scene
    objColl.operation = CollisionObject.ADD
    attach_object_publisher = rospy.Publisher('collision_object',CollisionObject,queue_size = 10)
    rospy.sleep(2)
    attach_object_publisher.publish(objColl)
    rospy.sleep(1)

def clean_scene():

    scene.remove_world_object("box_number1")

#Alternative method
def obstacle_AttachAdd():
    patternBlock = SolidPrimitive()
    patternBlock.type = SolidPrimitive.BOX
    #shape.dimensions.resize(3)
    patternBlock.dimensions=list((0.8,0.8,0.8))
    
    cube = PoseStamped()
    cube.pose.position.x=-0.3
    cube.pose.position.y=0.0
    cube.pose.position.z=0.29+0.15+0.075
    cube.pose.orientation.x=0
    cube.pose.orientation.y=0
    cube.pose.orientation.z=-math.sqrt(0.5)
    cube.pose.orientation.w=math.sqrt(0.5)
    size=(0.08, 0.08, 0.3)
    objColl = CollisionObject()
    objColl.header=cube.header
    objColl.primitives = [patternBlock]
    objColl.header.frame_id= "world" #controller.planning_frame
    objColl.header.stamp = rospy.Time.now()
    #objColl.id = 'cafe_table'

    objColl.primitive_poses=cube
    eef_link=controller.eeLink
    endEffector_group=controller.bodyLinks
    scene.add_box("box_number1", objColl.primitive_poses, size)
    #rospy.sleep(5)
    #scene.attach_box("box_number1",eef_link, touch_links=endEffector_group)


def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

  box_name = self.box_name
  scene = self.scene

  start = rospy.get_time()
  seconds = rospy.get_time()

  while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        rospy.sleep(0.1)
        seconds = rospy.get_time()

  return False

def createScene():
    controller = EdoFactory().create_robot_model(SIMULATION)
    scene=controller.scene


if __name__ == '__main__':
    rospy.init_node('avoid_obstacle')
    #createScene()
    rospy.sleep(5)
 
    #obstacleAdd(cube, patternBlock)
    #obstacle_AttachAdd(cube, patternBlock, size)
    obstacle_AttachAdd()
    rospy.sleep(10)

    clean_scene()
    # Ensuring Collision Updates Are Receieved
    #wait_for_state_update()
    rospy.spin()
