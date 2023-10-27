import socket		
import rospy
from sensor_msgs.msg import Imu
import math

def to_quaternion(roll, pitch, yaw):

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z

def imu_pub():
    
    rospy.init_node('imu_pub', anonymous=True)
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print ("Socket successfully created")

    host='192.168.68.132'
    port = 8765

    s.bind((host, port))		
    print ("socket binded to %s" %(port))

    s.listen(1)	
    print ("socket is listening")		

    c, addr = s.accept()	
    print ('Got connection from', addr)
        
    pub = rospy.Publisher('/imu_phone', Imu, queue_size=10)

    rate = rospy.Rate(10) 
    
    c.settimeout(5)

    while not rospy.is_shutdown():
      try:
        msg = c.recv(1024)
        if not msg:
          raise socket.timeout("No data recieved")
      except socket.timeout:
        print("Timeout occurred. No data received.")
        break

      data=msg.decode("utf-8")
      data = data.replace("\r", "").replace("\n", "")
      values=data.split(',')

      try:
        values.remove('')
      except ValueError:
         pass      
      
      if len(values)==10:
        print(values)
        time_nsec=int(values[0])
        secs = time_nsec // int(1e9)
        nsecs = time_nsec % int(1e9)

        time = rospy.Time(secs, nsecs)
        
        imu_msg=Imu()
        imu_msg.header.stamp = time
        imu_msg.header.frame_id = 'map'
        
        imu_msg.linear_acceleration.x = float(values[1])
        imu_msg.linear_acceleration.y = float(values[2])
        imu_msg.linear_acceleration.z = float(values[3])

        imu_msg.angular_velocity.x = float(values[4])
        imu_msg.angular_velocity.y = float(values[5])
        imu_msg.angular_velocity.z = float(values[6])

        try:
          w, x, y, z = to_quaternion(float(values[7]), float(values[8]), float(values[9]))
        except ValueError:
           continue

        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w

        pub.publish(imu_msg)

      else:
         continue

      rate.sleep()  

    c.close()
    

if __name__ == '__main__':
    try:
        imu_pub()
    except rospy.ROSInterruptException:
        pass




