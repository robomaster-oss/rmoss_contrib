#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import sys, select, tty, termios
from rm_interfaces.srv import SetMode

msg = """
task_auto_aim client test
Please input keys:
---------------------------
q,休眠模式，w:自动射击模式，e：自动瞄准模式（不发子弹）,r,测试模式,不控制.
---------------------------
（TODO）a:change to red color , s:change to blue color
---------------------------
CTRL-C to quit
"""

def set_mode_req(cli, mode):
    req = SetMode.Request()
    req.mode = mode
    srv_call = cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = Node("task_auto_aim_client_test")
    cli = node.create_client(SetMode, "task_auto_aim/set_mode")
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    print(msg)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    while rclpy.ok():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key=sys.stdin.read(1)
            print(key)
            if (key == 'q' or key == 'Q'):
                set_mode_req(cli,0x00)
            elif(key=='w' or key=='W'):
                set_mode_req(cli,0x01)
            elif(key=='e' or key=='E'):
                 set_mode_req(cli,0x02)
            elif(key=='r' or key=='R'):
                 set_mode_req(cli,0x03)
            elif(key=='a' or key=='A'):
                set_mode_req(cli,0x10)
            elif(key=='s' or key=='S'):
                set_mode_req(cli,0x11)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
