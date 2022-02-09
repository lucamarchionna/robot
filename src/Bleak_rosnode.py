#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32    #Message is single float number

import asyncio
import struct
from bleak import BleakClient   #Bluetooth library


ADDRESS = "A9:88:08:2C:E9:C6"   #Address of device, always the same for same board
nanoUUID="2A56"                 #UUID chosen inside Arduino sketch,16bit standard
charUUID="0000"+nanoUUID+"-0000-1000-8000-00805f9b34fb"             #Arduino standard full 128bit ID
nanoService="180C"              #Service ID chosen inside Arduino sketch,16bit standard
#serviceInfo="0000"+nanoService+"-0000-1000-8000-00805f9b34fb"     #Not used

async def publish_sensor(mac_addr: str):
    client = BleakClient(mac_addr)  #principal object for the BT connection
    try:
        await client.connect()  #connect to device with specific MAC, timeout of 10 seconds (blocking call)

        #If connected, initialize ROS node, message, topic
        forceMsg=Float32()
        pub = rospy.Publisher('/appliedForce', Float32, queue_size=10)
        rospy.init_node('sensor_publisher', anonymous=False)         

        while not rospy.is_shutdown():   #Trying a loop reading
            reading=bytes(await client.read_gatt_char(charUUID))    #read value in bytes, type agnostic, directly to characteristic
            value = round(struct.unpack('f',reading)[0], 2)         #convert info 2 decimals float, knowing the type
            #print(f" Reading: {reading}, Value: {value}")           #logging just to check
            forceMsg.data=value
            pub.publish(forceMsg)   #publish the sensor value
        else:
            await client.disconnect()                               #disconnect from device when ROS node is closed
    except Exception as e:
        print(f" Exception: {e}")
    finally:
        await client.disconnect()                               #disconnect from device to be sure


if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(publish_sensor(ADDRESS))
    except rospy.ROSInterruptException:
        pass