#!/usr/bin/python3
import rclpy
import sys, select, tty, termios
from rm_interfaces.srv import SetMode

def task_auto_aim_client(cmd,param=0x00):
    try:
        rospy.wait_for_service('task_auto_aim_set_mode',timeout=0.01)
        task_auto_aim_set_mode = rospy.ServiceProxy('task_auto_aim_set_mode', SetMode)
        res = task_auto_aim_set_mode(cmd,param)
        return res.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except rospy.exceptions.ROSException,e:
        print "timeout: %s"%e

if __name__ == '__main__':
	rospy.init_node('task_auto_aim_client_test')
	rate = rospy.Rate(100)
	old_attr = termios.tcgetattr(sys.stdin)  
	tty.setcbreak(sys.stdin.fileno())
	print('''task auto aim client: \nPlease input keys:
    q:normal start, w:start(No Launch), e:start(No Control)
    r:stop
    a:change to red color , s:change to blue color
    press Ctrl + C to quit\n''')
	while not rospy.is_shutdown():
		if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
			key=sys.stdin.read(1)
			if (key == 'q' or key == 'Q'):
				ret=task_auto_aim_client(0x00)
				print "result %r"%(ret)
			elif(key=='w' or key=='W'):
				ret=task_auto_aim_client(0x01)
				print "result %r"%(ret)
			elif(key=='e' or key=='E'):
 				ret=task_auto_aim_client(0x02)
				print "result %r"%(ret)
			elif(key=='r' or key=='R'):
 				ret=task_auto_aim_client(0x03)
				print "result %r"%(ret)
			elif(key=='a' or key=='A'):
 				ret=task_auto_aim_client(0x10,1)
				print "result %r"%(ret)
			elif(key=='s' or key=='S'):
 				ret=task_auto_aim_client(0x10,0)
				print "result %r"%(ret)
		rate.sleep() 
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr) 
