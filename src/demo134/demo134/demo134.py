# #!/usr/bin/env python3
# #
# #   demo134.py
# #
# #   Demonstration node to interact with the HEBIs.
# #
# import numpy as np
# import rclpy

# from rclpy.node         import Node
# from sensor_msgs.msg    import JointState


# #
# #   Definitions
# #
# RATE = 100.0            # Hertz


# #
# #   DEMO Node Class
# #
# class DemoNode(Node):
#     # Initialization.
#     def __init__(self, name):
#         # Initialize the node, naming it as specified
#         super().__init__(name)

#         # Create a temporary subscriber to grab the initial position.
#         self.position0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % self.position0)

#         # Create a message and publisher to send the joint commands.
#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         # means we don't start until the rest of the system is ready.
#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while(not self.count_subscribers('/joint_commands')):
#             pass

#         # Create a subscriber to continually receive joint state messages.
#         self.fbksub = self.create_subscription(
#             JointState, '/joint_states', self.recvfbk, 10)

#         # Create a timer to keep calculating/sending commands.
#         rate           = RATE
#         self.current_phase = "homing"
#         self.last_joint_positions = self.position0
#         # self.homing_time = 2.0
#         # self.homed = False
#         self.starttime = self.get_clock().now()
#         self.timer     = self.create_timer(1/rate, self.update)
#         self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
#                                (self.timer.timer_period_ns * 1e-9, rate))

#     # Shutdown
#     def shutdown(self):
#         # No particular cleanup, just shut down the node.
#         self.destroy_node()


#     # Grab a single feedback - DO NOT CALL THIS REPEATEDLY!
#     def grabfbk(self):
#         # Create a temporary handler to grab the position.
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         # Temporarily subscribe to get just one message.
#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)

#         # Return the values.
#         return self.grabpos

#     # Send a command.
#     def sendcmd(self, pos, vel, eff = []):
#         # Build up the message and publish.
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name         = ['one', 'two', 'three']
#         self.cmdmsg.position     = pos
#         self.cmdmsg.velocity     = vel
#         self.cmdmsg.effort       = eff
#         self.cmdpub.publish(self.cmdmsg)


#     ######################################################################
#     # Handlers
#     # Receive feedback - called repeatedly by incoming messages.
#     # def recvfbk(self, fbkmsg):
#     #     # Just print the position (for now).
#     #     # print(list(fbkmsg.position))
#     #     pass

#     def recvfbk(self, fbkmsg):
#         """Update the last known joint positions from feedback."""
#         self.last_joint_positions = list(fbkmsg.position)

#     def ikin(self, x, y, z):
#         """Compute joint angles from cartesian coordinates"""
#         r = np.sqrt(x**2 + y**2)
#         d = z
#         theta1 = np.arctan2(y, x)
#         D = np.sqrt(r**2 + d**2)
#         if D > (L1 + L2) or D < abs(L1 - L2):
#             raise ValueError("Target out of reach")

#         angle_a = np.arctan2(d, r)
#         angle_b = np.arccos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
#         theta2 = angle_a + angle_b

#         angle_c = np.arccos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
#         theta3 = np.pi - angle_c

#         return [theta1, theta2, theta3]

#     def move_with_spline(self, q_start, q_end, duration):
#         """Move between two joint positions using quintic spline."""
#         t_start = self.get_clock().now()
#         t_now = self.get_clock().now()
#         elapsed = (t_now - t_start).nanoseconds * 1e-9

#         while elapsed < duration:
#             t_now = self.get_clock().now()
#             elapsed = (t_now - t_start).nanoseconds * 1e-9
#             pos, vel, _ = self.quintic_spline(q_start, q_end, duration, elapsed)
#             self.sendcmd(pos, vel)
#             rclpy.spin_once(self)
            
#         # Update last known positions - not sure whether to do here or in the update function
#         self.last_joint_positions = q_end

#     def move_to_waiting_state(self):
#         # TODO: find fixed waiting pos. 
#         """Move robot to the waiting state using quintic spline."""
#         current_pos = self.last_joint_positions  
#         # Step 1: Move upper arm vertical
#         intermediate_pos = [current_pos[0], 0.0, current_pos[2]]
#         self.move_with_spline(current_pos, intermediate_pos, 2.0)

#         # Step 2: Move pointer to default waiting position
#         waiting_pos = [0.0, 0.0, np.pi / 2]  # We should change this once we know the exact position of our waiting state
#         self.move_with_spline(intermediate_pos, waiting_pos, 2.0)  

#     def quintic_spline(self, q0, qT, T, t):
#         """
#         Compute the position, velocity, and acceleration using a quintic spline.
#         - param q0: Vector of initial positions
#         - param qT: Vector of final positions
#         - param T:  Total time for the motion
#         - param t:  Current time
#         - return: Vector: positions, Vector: velocities, Vector: accelerations 
#         """
#         positions, velocities, accelerations = [], [], []
    
#         for i in range(len(q0)):
#             a0 = q0[i]
#             a1 = 0.0
#             a2 = 0.0
#             a3 = (20 * (qT[i] - q0[i]) - (8 * 0.0 + 12 * 0.0) * T - (3 * 0.0 - 0.0) * T**2) / (2 * T**3)
#             a4 = (-30 * (qT[i] - q0[i]) + (14 * 0.0 + 16 * 0.0) * T + (3 * 0.0 - 2 * 0.0) * T**2) / (2 * T**4)
#             a5 = (12 * (qT[i] - q0[i]) - (6 * 0.0 + 6 * 0.0) * T - (0.0 - 0.0) * T**2) / (2 * T**5)
    
#             position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
#             velocity = a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
#             acceleration = 2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    
#             positions.append(position)
#             velocities.append(velocity)
#             accelerations.append(acceleration)
    
#         return positions, velocities, accelerations

#     def move_to_target(self, x, y, z):
#         """Move robot to a target point on the table directly using quintic spline."""
#         try:
#             joint_angles = self.ikin(x, y, z)
#             # Move directly to the target - to move to target we do not need a 2 step process. 
#             self.move_with_spline(self.last_joint_positions, joint_angles, 2.0)
#         except ValueError as e:
#             self.get_logger().error(str(e))

#     def update(self):
#         """Main update loop."""
#         # TODO: set up a way to move between the states - decide on logic on how to move between the states.
#         if self.current_phase == "startup":
#             self.move_to_waiting_state()
#             self.current_phase = "moving"

#         elif self.current_phase == "moving":
#             x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
#             self.move_to_target(x, y, z)
#             self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             self.move_to_waiting_state()
#             self.current_phase = "waiting"


#     # # Timer (100Hz) update.
#     # def update(self):
#     #     # Grab the current time.
#     #     now = self.get_clock().now()
#     #     t   = (now - self.starttime).nanoseconds * 1e-9

#     #     if self.current_phase == "homing":
#     #         progress = min(t / self.homing_time, 1.0)  
#     #         qd = [(1 - progress) * p0 for p0 in self.position0]
#     #         qddot = [0.0, 0.0, 0.0]
#     #         self.sendcmd(qd, qddot)

#     #         # code to use the quintic spline instead of linear interpolation
#     #         # T = self.homing_time
#     #         # qd, qddot, qddotdot = self.quintic_spline(self.position0, [0.0, 0.0, 0.0], T, t)
#     #         # self.sendcmd(qd, qddot)

#     #         if progress >= 1.0:
#     #         # if t + dt >= 1.0
#     #             self.current_phase = "waving"
#     #             self.starttime = now  
#     #             self.get_logger().info("Homing complete. Starting waving motion.")

#     #     # Compute the trajectory.
#     #     # qd    = [1.0, 2.0, 3.0]
#     #     elif self.current_phase == "waving":
#     #         qd = [0.0, 0.2 * np.sin(2 * np.pi * 0.1 * t), 0.5 * np.sin(2 * np.pi * 0.5 * t)]
#     #         qddot = [0.0, 0.0, 0.0]

#     #         # Send.
#     #         self.sendcmd(qd, qddot)


# #
# #   Main Code
# #
# def main(args=None):
#     # Initialize ROS.
#     rclpy.init(args=args)

#     # Instantiate the DEMO node.
#     node = DemoNode('demo')

#     # Spin the node until interrupted.
#     rclpy.spin(node)

#     # Shutdown the node and ROS.
#     node.shutdown()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

from ikpy.chain import Chain
import ikpy

RATE = 100.0  # Hertz

class DemoNode(Node):
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # self.robot_chain = Chain.from_urdf_file("src/threedof/threedof/urdf/threedofexample.urdf", base_elements=["world"])

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
        self.current_phase = "waiting"
        self.last_joint_positions = self.position0
        self.homing_time = 2.0
        # self.homed = False
        self.starttime = self.get_clock().now()
        self.timer     = self.create_timer(1/rate, self.update)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        self.q_current = self.position0

    def quintic_spline(self, q0, qT, T, t):
        """Compute position and velocity for a scalar value using a quintic spline."""
        a0 = q0
        a1 = 0.0
        a2 = 0.0
        a3 = (10*(qT - q0)) / (T**3)
        a4 = (-15*(qT - q0)) / (T**4)
        a5 = (6*(qT - q0)) / (T**5)

        position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        return position, velocity
    
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
    
    def recvfbk(self, fbkmsg):
        # Save the actual position.
        self.actpos = fbkmsg.position

    def cartesian_to_joint_space(self, x, y, z):
        # Link lengths (assumed from URDF for simplification)
        L1 = 0.385  # shoulder to elbow length
        L2 = 0.37   # elbow to tip length

        # Compute the base rotation angle (q1) about the z-axis
        q1 = np.arctan2(y, x)

        # Project the target onto the plane defined by the shoulder and elbow after base rotation
        r = np.sqrt(x**2 + y**2)  # horizontal distance from base axis to target

        # Using planar 2R arm inverse kinematics in the plane defined by (r, z)
        # Compute intermediate value for elbow angle calculation using the law of cosines
        D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

        # Check if the target is reachable
        if abs(D) > 1:
            raise ValueError("Target is unreachable with the given arm configuration.")

        # Elbow joint angle (q3) solution (elbow-up configuration assumed)
        q3 = np.arccos(D)

        # Compute the angle from the horizontal to the line connecting shoulder to target
        phi = np.arctan2(z, r)

        # Angle between link L1 and the line from shoulder to target
        psi = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))

        # Shoulder joint angle (q2)
        q2 = phi - psi

        return [q1, q2, q3]

    def move_with_spline(self, q_start, q_goal, duration):
        """Moves joints from q_start to q_goal over the given duration using a quintic spline."""
        start_time = self.get_clock().now()
        rate = self.create_rate(RATE)

        while rclpy.ok():
            # Calculate elapsed time
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds * 1e-9

            # Clamp elapsed time to duration
            t = min(elapsed, duration)

            # Compute new positions and velocities for each joint
            positions = []
            velocities = []
            for q0, qT in zip(q_start, q_goal):
                p, v = self.quintic_spline(q0, qT, duration, t)
                positions.append(p)
                velocities.append(v)

            # Send command
            self.sendcmd(positions, velocities)

            # Break loop if motion complete
            if t >= duration:
                self.sendcmd(positions, [0.0]*len(velocities))
                self.q_current = q_goal  # Update current joint state
                break

    def sendcmd(self, positions, velocities):
        """Sends joint commands."""
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = ['base', 'shoulder', 'elbow']  # Adjust joint names if necessary
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        self.cmdpub.publish(self.cmdmsg)

    def move_to_waiting_position(self):
        """Directly move robot joints to the waiting position using a spline."""
        duration = 2.0
        intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
        self.move_with_spline(self.q_current, intermediate_pos, duration)
        waiting_position = [np.pi/2, 0.0, np.pi/2]  
        self.move_with_spline(self.q_current, waiting_position, duration)
        self.get_logger().info("Reached waiting position.")

    def update(self):
        if self.current_phase == "waiting":
            self.move_to_waiting_position()
            self.current_phase = "moving"

        elif self.current_phase == "moving":
            x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
            q_target = self.cartesian_to_joint_space(x, y, z)
            # self.move_with_spline(self.q_current, [np.pi, np.pi/2, 0.0], 2.0)
            self.move_with_spline(self.q_current, q_target, 2.0)
            self.current_phase = "returning"

        elif self.current_phase == "returning":
            self.move_to_waiting_position()
            self.current_phase = "waiting"

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('demo')
    try:
        rclpy.spin(node)  # Keeps spinning until shutdown is requested
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
