#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate           = RATE
        self.current_phase = "homing"
        self.last_joint_positions = self.position0
        # self.homing_time = 2.0
        # self.homed = False
        self.starttime = self.get_clock().now()
        self.timer     = self.create_timer(1/rate, self.update)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Grab a single feedback - DO NOT CALL THIS REPEATEDLY!
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos

    # Send a command.
    def sendcmd(self, pos, vel, eff = []):
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['one', 'two', 'three']
        self.cmdmsg.position     = pos
        self.cmdmsg.velocity     = vel
        self.cmdmsg.effort       = eff
        self.cmdpub.publish(self.cmdmsg)


    ######################################################################
    # Handlers
    # Receive feedback - called repeatedly by incoming messages.
    # def recvfbk(self, fbkmsg):
    #     # Just print the position (for now).
    #     # print(list(fbkmsg.position))
    #     pass

    def recvfbk(self, fbkmsg):
        """Update the last known joint positions from feedback."""
        self.last_joint_positions = list(fbkmsg.position)

    def ikin(self, x, y, z):
        """Compute joint angles from cartesian coordinates"""
        r = np.sqrt(x**2 + y**2)
        d = z
        theta1 = np.arctan2(y, x)
        D = np.sqrt(r**2 + d**2)
        if D > (L1 + L2) or D < abs(L1 - L2):
            raise ValueError("Target out of reach")

        angle_a = np.arctan2(d, r)
        angle_b = np.arccos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        theta2 = angle_a + angle_b

        angle_c = np.arccos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
        theta3 = np.pi - angle_c

        return [theta1, theta2, theta3]

    def move_with_spline(self, q_start, q_end, duration):
        """Move between two joint positions using quintic spline."""
        t_start = self.get_clock().now()
        t_now = self.get_clock().now()
        elapsed = (t_now - t_start).nanoseconds * 1e-9

        while elapsed < duration:
            t_now = self.get_clock().now()
            elapsed = (t_now - t_start).nanoseconds * 1e-9
            pos, vel, _ = self.quintic_spline(q_start, q_end, duration, elapsed)
            self.sendcmd(pos, vel)
            rclpy.spin_once(self)
            
        # Update last known positions - not sure whether to do here or in the update function
        self.last_joint_positions = q_end

    def move_to_waiting_state(self):
        # TODO: find fixed waiting pos. 
        """Move robot to the waiting state using quintic spline."""
        current_pos = self.last_joint_positions  
        # Step 1: Move upper arm vertical
        intermediate_pos = [current_pos[0], 0.0, current_pos[2]]
        self.move_with_spline(current_pos, intermediate_pos, 2.0)

        # Step 2: Move pointer to default waiting position
        waiting_pos = [0.0, 0.0, np.pi / 2]  # We should change this once we know the exact position of our waiting state
        self.move_with_spline(intermediate_pos, waiting_pos, 2.0)  

    def quintic_spline(self, q0, qT, T, t):
        """
        Compute the position, velocity, and acceleration using a quintic spline.
        - param q0: Vector of initial positions
        - param qT: Vector of final positions
        - param T:  Total time for the motion
        - param t:  Current time
        - return: Vector: positions, Vector: velocities, Vector: accelerations 
        """
        positions, velocities, accelerations = [], [], []
    
        for i in range(len(q0)):
            a0 = q0[i]
            a1 = 0.0
            a2 = 0.0
            a3 = (20 * (qT[i] - q0[i]) - (8 * 0.0 + 12 * 0.0) * T - (3 * 0.0 - 0.0) * T**2) / (2 * T**3)
            a4 = (-30 * (qT[i] - q0[i]) + (14 * 0.0 + 16 * 0.0) * T + (3 * 0.0 - 2 * 0.0) * T**2) / (2 * T**4)
            a5 = (12 * (qT[i] - q0[i]) - (6 * 0.0 + 6 * 0.0) * T - (0.0 - 0.0) * T**2) / (2 * T**5)
    
            position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
            velocity = a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
            acceleration = 2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    
            positions.append(position)
            velocities.append(velocity)
            accelerations.append(acceleration)
    
        return positions, velocities, accelerations

    def move_to_target(self, x, y, z):
        """Move robot to a target point on the table directly using quintic spline."""
        try:
            joint_angles = self.ikin(x, y, z)
            # Move directly to the target - to move to target we do not need a 2 step process. 
            self.move_with_spline(self.last_joint_positions, joint_angles, 2.0)
        except ValueError as e:
            self.get_logger().error(str(e))

    def update(self):
        """Main update loop."""
        # TODO: set up a way to move between the states - decide on logic on how to move between the states.
        if self.current_phase == "startup":
            self.move_to_waiting_state()
            self.current_phase = "moving"

        elif self.current_phase == "moving":
            x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
            self.move_to_target(x, y, z)
            self.current_phase = "returning"

        elif self.current_phase == "returning":
            self.move_to_waiting_state()
            self.current_phase = "waiting"


    # # Timer (100Hz) update.
    # def update(self):
    #     # Grab the current time.
    #     now = self.get_clock().now()
    #     t   = (now - self.starttime).nanoseconds * 1e-9

    #     if self.current_phase == "homing":
    #         progress = min(t / self.homing_time, 1.0)  
    #         qd = [(1 - progress) * p0 for p0 in self.position0]
    #         qddot = [0.0, 0.0, 0.0]
    #         self.sendcmd(qd, qddot)

    #         # code to use the quintic spline instead of linear interpolation
    #         # T = self.homing_time
    #         # qd, qddot, qddotdot = self.quintic_spline(self.position0, [0.0, 0.0, 0.0], T, t)
    #         # self.sendcmd(qd, qddot)

    #         if progress >= 1.0:
    #         # if t + dt >= 1.0
    #             self.current_phase = "waving"
    #             self.starttime = now  
    #             self.get_logger().info("Homing complete. Starting waving motion.")

    #     # Compute the trajectory.
    #     # qd    = [1.0, 2.0, 3.0]
    #     elif self.current_phase == "waving":
    #         qd = [0.0, 0.2 * np.sin(2 * np.pi * 0.1 * t), 0.5 * np.sin(2 * np.pi * 0.5 * t)]
    #         qddot = [0.0, 0.0, 0.0]

    #         # Send.
    #         self.sendcmd(qd, qddot)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
