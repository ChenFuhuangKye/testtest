import time
from rclpy.node import Node
import rclpy
from rclpy.duration import Duration
import orjson
from std_msgs.msg import String
import curses
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from pros_crane_py.env_virtual import *

class CraneKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 10):
        super().__init__('crane_keyboard')        

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            CRANE_STATE, # topic name
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            CRANE_CONTROL,  # topic name
            10
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, ARM_CONTROL, 10)
        self.joint_pos = [1.57, 1.57, 1.57, 1.57]
        self.stdscr = stdscr
        
        self.key_in_count = 0
        
    def _sub_callback(self, msg):        
        self._car_state_msg = str(self.get_clock().now()) + " " + msg.data

    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()
                self.movement = [0, 0, 0]

                # Check if a key was actually pressed
                if c != curses.ERR:
                    if c == ord('w') or c == ord('W'):
                        self.movement = [0, 1, 0]                    
                    elif c == ord('s') or c == ord('S'):
                        self.movement = [0, -1, 0]
                    elif c == ord('a') or c == ord('A'):
                        self.movement = [1, 0, 0]
                    elif c == ord('d') or c == ord('D'):
                        self.movement = [-1, 0, 0]
                    elif c == ord('z') or c == ord('Z'):
                        self.movement = [0, 0, 1]
                    elif c == ord('c') or c == ord('C'):
                        self.movement = [0, 0, -1]
                    elif c == ord('i'):
                        self.handle_key_i()
                    elif c == ord('j'):
                        self.handle_key_j()
                    elif c == ord('k'):
                        self.handle_key_k()
                    elif c == ord('l'):
                        self.handle_key_l()
                    elif c == ord('u'):
                        self.handle_key_u()
                    elif c == ord('o'):
                        self.handle_key_o()
                    elif c == ord('y'):
                        self.handle_key_y()
                    elif c == ord('h'):
                        self.handle_key_h()
                    elif c == ord('b'):
                        self.handle_key_b()
                    elif c == ord('q'):
                        self.pub_crane_control(self.movement)
                        break
                    
                    self.pub_crane_control(self.movement)
                    self.pub_arm()

                    # print the key pressed
                    self.key_in_count += 1
                    self.print_basic_info(c)
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.1)
                
        finally:
            curses.endwin()
    
    def print_basic_info(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"{self.movement}")
        self.stdscr.move(2, 0)
        self.stdscr.addstr(f"{self.joint_pos}")

    def pub_arm(self):
        msg = JointTrajectoryPoint()
        msg.positions = self.joint_pos
        msg.velocities = [0.0, 0.0, 0.0, 0.0]

        self.joint_trajectory_publisher_.publish(msg)

    def pub_crane_control(self, movement):
        # Generate a random control signal
        control_signal = {
            "type": CRANE_CONTROL,
            "data": dict(CraneMovementControl(
                moveX = movement[0],
                moveY = movement[1],
                moveZ = movement[2]
            ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        # self.get_logger().info(f'publish {control_msg}')
    
    def handle_key_w(self, movement: list):
        movement = [0, 1, 0]  
        self.stdscr.addstr(f"move forward")
        pass   

    def handle_key_i(self):
        self.stdscr.addstr(f"arm rift up")
        self.joint_pos[2] += 0.05
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"arm turn left")
        self.joint_pos[0] -= 0.05
        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"arm rift down")
        self.joint_pos[2] -= 0.05
        pass

    def handle_key_l(self):
        self.stdscr.addstr(f"arm turn right")
        self.joint_pos[0] += 0.05
        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"arm j4 rotate left")
        self.joint_pos[3] -= 0.05
        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"arm j4 rotate right")
        self.joint_pos[3] += 0.05
        pass
    
    def handle_key_y(self):
        self.stdscr.addstr(f"arm j2 up")
        self.joint_pos[1] += 0.05
        pass

    def handle_key_h(self):
        self.stdscr.addstr(f"arm j2 down")
        self.joint_pos[1] -= 0.05
        pass

    def handle_key_b(self):
        # 初始化機器手臂到預設位置的方法
        self.stdscr.addstr(f"將機器手臂初始化到預設位置...")
        # self.joint_pos = [0.0, 1.57, 1.57, 0.52,]  # 以弧度表示的角度（0, 90, 90, 0）
        # self.pub_arm()
        self.joint_pos = [math.radians(90), math.radians(90), math.radians(90), math.radians(0)]

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = CraneKeyboardController(stdscr)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == '__main__':
    main()