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


""" Working code """
# import numpy as np
# import rclpy

# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Point 

# from ikpy.chain import Chain
# import ikpy

# RATE = 100.0  # Hertz

# class DemoNode(Node):
#     def __init__(self, name):
#         # Initialize the node, naming it as specified
#         super().__init__(name)

#         # self.robot_chain = Chain.from_urdf_file("src/threedof/threedof/urdf/threedofexample.urdf", base_elements=["world"])
#         self.point_queue = []
#         self.pointsub = self.create_subscription(
#             Point, '/point', self.recvpoint, 10)
#         self.current_target = None
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
#         self.current_phase = "startup"
#         self.last_joint_positions = self.position0
#         self.homing_time = 2.0
#         # self.homed = False
#         self.starttime = self.get_clock().now()
#         self.timer     = self.create_timer(1/rate, self.update)
#         self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
#                                (self.timer.timer_period_ns * 1e-9, rate))
#         self.q_current = self.position0

#     def quintic_spline(self, q0, qT, T, t):
#         """Compute position and velocity for a scalar value using a quintic spline."""
#         a0 = q0
#         a1 = 0.0
#         a2 = 0.0
#         a3 = (10*(qT - q0)) / (T**3)
#         a4 = (-15*(qT - q0)) / (T**4)
#         a5 = (6*(qT - q0)) / (T**5)

#         position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
#         velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
#         return position, velocity
    
#     def recvpoint(self, pointmsg):
#         # Extract coordinates from message and enqueue them
#         point = (pointmsg.x, pointmsg.y, pointmsg.z)
#         self.point_queue.append(point)
#         self.get_logger().info(f"Received point: {point}")

    
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
    
#     def recvfbk(self, fbkmsg):
#         # Save the actual position.
#         self.actpos = fbkmsg.position

#     def cartesian_to_joint_space(self, x, y, z):
#         # Link lengths (assumed from URDF for simplification)
#         L1 = 0.385  # shoulder to elbow length
#         L2 = 0.37   # elbow to tip length

#         # Compute the base rotation angle (q1) about the z-axis
#         q1 = np.arctan2(y, x)

#         # Project the target onto the plane defined by the shoulder and elbow after base rotation
#         r = np.sqrt(x**2 + y**2)  # horizontal distance from base axis to target

#         # Using planar 2R arm inverse kinematics in the plane defined by (r, z)
#         # Compute intermediate value for elbow angle calculation using the law of cosines
#         D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

#         # Check if the target is reachable
#         if abs(D) > 1:
#             raise ValueError("Target is unreachable with the given arm configuration.")

#         # Elbow joint angle (q3) solution (elbow-up configuration assumed)
#         q3 = np.arccos(D)

#         # Compute the angle from the horizontal to the line connecting shoulder to target
#         phi = np.arctan2(z, r)

#         # Angle between link L1 and the line from shoulder to target
#         psi = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))

#         # Shoulder joint angle (q2)
#         q2 = phi - psi

#         return [q1, q2, q3]

#     def move_with_spline(self, q_start, q_goal, duration):
#         """Moves joints from q_start to q_goal over the given duration using a quintic spline."""
#         start_time = self.get_clock().now()
#         rate = self.create_rate(RATE)

#         while rclpy.ok():
#             # Calculate elapsed time
#             now = self.get_clock().now()
#             elapsed = (now - start_time).nanoseconds * 1e-9

#             # Clamp elapsed time to duration
#             t = min(elapsed, duration)

#             # Compute new positions and velocities for each joint
#             positions = []
#             velocities = []
#             for q0, qT in zip(q_start, q_goal):
#                 p, v = self.quintic_spline(q0, qT, duration, t)
#                 positions.append(p)
#                 velocities.append(v)

#             # Send command
#             self.sendcmd(positions, velocities)

#             # Break loop if motion complete
#             if t >= duration:
#                 self.sendcmd(positions, [0.0]*len(velocities))
#                 self.q_current = q_goal  # Update current joint state
#                 break

#     def sendcmd(self, positions, velocities):
#         """Sends joint commands."""
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name = ['base', 'shoulder', 'elbow']  # Adjust joint names if necessary
#         self.cmdmsg.position = positions
#         self.cmdmsg.velocity = velocities
#         self.cmdpub.publish(self.cmdmsg)

#     def move_to_waiting_position(self):
#         """Directly move robot joints to the waiting position using a spline."""
#         duration = 2.0
#         intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
#         self.move_with_spline(self.q_current, intermediate_pos, duration)
#         waiting_position = [np.pi/2, 0.0, np.pi/2]  
#         self.move_with_spline(self.q_current, waiting_position, duration)
#         self.get_logger().info("Reached waiting position.")

#     def update(self):
#         if self.current_phase == "startup":
#             self.move_to_waiting_position()
#             self.current_phase = "waiting"
                    
#         elif self.current_phase == "waiting":
#             # Only start moving if there is a new point in the queue
#             if self.point_queue:
#                 # Dequeue next point and transition to moving phase
#                 self.current_target = self.point_queue.pop(0)
#                 self.current_phase = "moving"
#             # If no new points, remain in waiting phase without action

#         elif self.current_phase == "moving":
#             # x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
#             # q_target = self.cartesian_to_joint_space(x, y, z)
#             # # self.move_with_spline(self.q_current, [np.pi, np.pi/2, 0.0], 2.0)
#             # self.move_with_spline(self.q_current, q_target, 2.0)
#             # self.current_phase = "returning"
#             x, y, z = self.current_target
#             try:
#                 q_target = self.cartesian_to_joint_space(x, y, z)
#             except ValueError:
#                 # Skip unreachable point and return to waiting phase
#                 self.get_logger().info("Target unreachable, skipping this point.")
#                 self.current_phase = "waiting"
#                 return

#             self.move_with_spline(self.q_current, q_target, 2.0)
#             self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             self.move_to_waiting_position()
#             self.current_phase = "waiting"

# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoNode('demo')
#     try:
#         rclpy.spin(node)  # Keeps spinning until shutdown is requested
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


# """Code to try using kinematic chain"""
# import numpy as np
# import rclpy

# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Point

# # KinematicChain import
# from KinematicChain import KinematicChain
# from KinematicChain import JointType  # might not be used directly, but for reference

# RATE = 100.0  # Hertz

# class DemoNode(Node):
#     def __init__(self, name):
#         # Initialize the node, naming it as specified
#         super().__init__(name)

#         # Initialize the Kinematic Chain
#         # ---------------------------------------------------------
#         # Adjust the baseframe and tipframe to match your URDF 
#         # as needed. The final list is the expected active joint names 
#         # in correct order (the same names used in your URDF).
#         # ---------------------------------------------------------
#         base_frame = "world"
#         tip_frame  = "tip"  # or whatever your URDF calls the last link
#         expected_joint_names = ["base", "shoulder", "elbow"]
#         self.chain = KinematicChain(
#             node=self,
#             baseframe=base_frame,
#             tipframe=tip_frame,
#             expectedjointnames=expected_joint_names
#         )
#         # Gain for IK solver
#         self.lam = 20.0  

#         # For storing the actual position & for printing
#         self.actpos = [0.0, 0.0, 0.0]

#         # Subscribe to the point (Cartesian target) messages
#         self.point_queue = []
#         self.pointsub = self.create_subscription(
#             Point, '/point', self.recvpoint, 10)

#         self.current_target = None

#         # Grab the initial joint positions
#         self.position0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % self.position0)

#         # Create a message and publisher to send the joint commands.
#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         # it means we don't start until the rest of the system is ready.
#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while(not self.count_subscribers('/joint_commands')):
#             pass

#         # Create a subscriber to continually receive joint state messages.
#         self.fbksub = self.create_subscription(
#             JointState, '/joint_states', self.recvfbk, 10)

#         # Set up our timer for the main control loop
#         self.current_phase = "startup"
#         self.q_current     = self.position0
#         self.starttime     = self.get_clock().now()
#         self.timer         = self.create_timer(1.0/RATE, self.update)

#         self.get_logger().info("Sending commands at %f Hz" % RATE)

#     def recvpoint(self, pointmsg):
#         """Receive a Cartesian target point and enqueue it."""
#         point = (pointmsg.x, pointmsg.y, pointmsg.z)
#         self.point_queue.append(point)
#         self.get_logger().info(f"Received point: {point}")

#     def grabfbk(self):
#         """Grab one set of feedback from the '/joint_states' topic (blocking)."""
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)

#         return self.grabpos

#     def recvfbk(self, fbkmsg):
#         """Callback to continually receive feedback from '/joint_states'."""
#         self.actpos = list(fbkmsg.position)

#     def sendcmd(self, positions, velocities):
#         """Sends joint commands."""
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         # Make sure the joint names match your URDF joints:
#         self.cmdmsg.name     = ['base', 'shoulder', 'elbow']
#         self.cmdmsg.position = positions
#         self.cmdmsg.velocity = velocities
#         self.cmdpub.publish(self.cmdmsg)

#     #
#     # Quintic spline for an entire multi-joint segment
#     #
#     def quintic_spline(self, q0, qT, T, t):
#         """
#         Compute array of positions and velocities for each DOF 
#         using a quintic polynomial between q0 and qT. 
#         q0, qT are arrays of the same length.
#         """
#         # Make sure q0, qT are arrays
#         q0 = np.array(q0)
#         qT = np.array(qT)
#         positions = []
#         velocities = []
#         for i in range(len(q0)):
#             # For each DOF, compute the quintic
#             a0 = q0[i]
#             a1 = 0.0
#             a2 = 0.0
#             a3 = (10*(qT[i] - q0[i])) / (T**3)
#             a4 = (-15*(qT[i] - q0[i])) / (T**4)
#             a5 = (6*(qT[i] - q0[i])) / (T**5)

#             p = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
#             v = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)

#             positions.append(p)
#             velocities.append(v)

#         return positions, velocities

#     #
#     # Use the KinematicChain object to solve for joint angles
#     # that bring the tip to a desired Cartesian location [x, y, z].
#     #
#     def cartesian_to_joint_space_ik(self, x, y, z, q_init=None, tol=1e-3, max_iter=100):
#         """
#         Simple numerical IK solver ignoring orientation:
#           - p_des = [x, y, z]
#           - Start from q_init (if None, use self.q_current)
#           - Use chain.fkin(q) to get (ptip, Rtip, Jv, Jw).
#           - Update q with: q <- q + lam * pinv(Jv) * (p_des - ptip).
#           - Stop if norm(p_des - ptip) < tol or iteration count reached.
#         """
#         p_des = np.array([x, y, z])

#         if q_init is None:
#             q = np.array(self.q_current, dtype=float)
#         else:
#             q = np.array(q_init, dtype=float)

#         for _ in range(max_iter):
#             # Forward kinematics
#             ptip, Rtip, Jv, Jw = self.chain.fkin(q)
#             ptip = np.array(ptip)

#             # Position error
#             e = p_des - ptip
#             err_norm = np.linalg.norm(e)

#             if err_norm < tol:
#                 break

#             # Compute pseudo-inverse of Jv
#             # If your manipulator is 3-DOF and you only care about x,y,z,
#             # Jv is 3x3. We can invert or use np.linalg.pinv:
#             Jv_pinv = np.linalg.pinv(Jv)  

#             # Update rule
#             dq = self.lam * (Jv_pinv @ e)
#             q  = q + dq

#         # Return final q
#         return list(q)

#     #
#     # Move from current q to a new q over "duration" seconds via quintic splines
#     #
#     def move_with_spline(self, q_start, q_goal, duration):
#         """Moves joints from q_start to q_goal over the given duration."""
#         start_time = self.get_clock().now()

#         while rclpy.ok():
#             now     = self.get_clock().now()
#             elapsed = (now - start_time).nanoseconds * 1e-9

#             # Clamp elapsed time to duration
#             t = min(elapsed, duration)

#             # Compute new positions and velocities
#             positions, velocities = self.quintic_spline(q_start, q_goal, duration, t)

#             # Send the command
#             self.sendcmd(positions, velocities)

#             # Break loop if motion complete
#             if t >= duration:
#                 # Once done, send zero velocity to hold final position
#                 self.sendcmd(positions, [0.0]*len(velocities))
#                 self.q_current = q_goal  # Update internal "current" config
#                 break

#     #
#     # Move to a "waiting position"
#     #
#     def move_to_waiting_position(self):
#         duration = 2.0
#         # Example: intermediate then final:
#         intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
#         self.move_with_spline(self.q_current, intermediate_pos, duration)

#         # Example waiting position
#         waiting_position = [np.pi/2, 0.0, np.pi/2]
#         self.move_with_spline(intermediate_pos, waiting_position, duration)

#         self.get_logger().info("Reached waiting position.")

#     #
#     # Main update step
#     #
#     def update(self):
#         if self.current_phase == "startup":
#             # Move from wherever we are to a "waiting" posture
#             self.move_to_waiting_position()
#             self.current_phase = "waiting"

#         elif self.current_phase == "waiting":
#             # Only move if there's a new point
#             if self.point_queue:
#                 self.current_target = self.point_queue.pop(0)
#                 self.current_phase  = "moving"
#             else:
#                 # No new target, do nothing
#                 pass

#         elif self.current_phase == "moving":
#             x, y, z = self.current_target
#             # Solve IK numerically via chain
#             try:
#                 q_target = self.cartesian_to_joint_space_ik(x, y, z)
#             except np.linalg.LinAlgError:
#                 self.get_logger().info("Numerical error in IK. Skipping target.")
#                 self.current_phase = "waiting"
#                 return

#             # (Optionally check if solution is 'too large', etc.)

#             # Now move to the solution
#             self.move_with_spline(self.q_current, q_target, 2.0)

#             self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             self.move_to_waiting_position()
#             self.current_phase = "waiting"

# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoNode('demo')
#     try:
#         rclpy.spin(node)  # Keeps spinning until shutdown is requested
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# KinematicChain import
from threedof.KinematicChain import KinematicChain
from threedof.KinematicChain import JointType  # might not be used directly, but for reference
from threedof.TrajectoryUtils import *

RATE = 100.0  # Hertz

class DemoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        base_frame = "world"
        tip_frame  = "tip"  # or whatever your URDF calls the last link
        expected_joint_names = ["base", "shoulder", "elbow"]
        self.chain = KinematicChain(
            node=self,
            baseframe=base_frame,
            tipframe=tip_frame,
            expectedjointnames=expected_joint_names
        )
        self.lam = 20.0 
        self.max_iter = 100
        self.tol = 1e-3
        self.e = np.zeros(3)

        self.point_queue = []
        self.pointsub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)

        self.current_target = None

        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Set up our timer for the main control loop
        self.current_phase = "startup"
        self.q_current     = self.position0
        self.starttime     = self.get_clock().now()
        self.timer         = self.create_timer(1.0/RATE, self.update)

        self.get_logger().info("Sending commands at %f Hz" % RATE)
        self.x = self.chain.fkin(self.q_current)

    def recvpoint(self, pointmsg):
        """Receive a Cartesian target point and enqueue it."""
        x = pointmsg.x
        y = pointmsg.y
        z = pointmsg.z
        point = (x, y, z)
        self.point_queue.append(point)
        self.get_logger().info(f"Received point: {point}")

    def grabfbk(self):
        """Grab one set of feedback from the '/joint_states' topic (blocking)."""
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        return self.grabpos

    def recvfbk(self, fbkmsg):
        """Callback to continually receive feedback from '/joint_states'."""
        self.actpos = list(fbkmsg.position)

    def sendcmd(self, positions, velocities):
        """Sends joint commands."""
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name     = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        self.cmdpub.publish(self.cmdmsg)


    def quintic_spline(self, q0, qT, T, t):
        """
        Compute array of positions and velocities for each DOF 
        using a quintic polynomial between q0 and qT. 
        q0, qT are arrays of the same length.
        """
        # Make sure q0, qT are arrays
        q0 = np.array(q0)
        qT = np.array(qT)
        positions = []
        velocities = []
        for i in range(len(q0)):
            # For each DOF, compute the quintic
            a0 = q0[i]
            a1 = 0.0
            a2 = 0.0
            a3 = (10*(qT[i] - q0[i])) / (T**3)
            a4 = (-15*(qT[i] - q0[i])) / (T**4)
            a5 = (6*(qT[i] - q0[i])) / (T**5)

            p = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
            v = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)

            positions.append(p)
            velocities.append(v)

        return positions, velocities

    #
    # Use the KinematicChain object to solve for joint angles
    # that bring the tip to a desired Cartesian location [x, y, z].
    #
    # def cartesian_to_joint_space_ik(self, x, y, z, q_init=None, tol=1e-3, max_iter=100):
    #     """
    #     Simple numerical IK solver ignoring orientation:
    #       - p_des = [x, y, z]
    #       - Start from q_init (if None, use self.q_current)
    #       - Use chain.fkin(q) to get (ptip, Rtip, Jv, Jw).
    #       - Update q with: q <- q + lam * pinv(Jv) * (p_des - ptip).
    #       - Stop if norm(p_des - ptip) < tol or iteration count reached.
    #     """
    #     p_des = np.array([x, y, z])

    #     if q_init is None:
    #         q = np.array(self.q_current, dtype=float)
    #     else:
    #         q = np.array(q_init, dtype=float)

    #     for _ in range(max_iter):
    #         # Forward kinematics
    #         ptip, Rtip, Jv, Jw = self.chain.fkin(q)
    #         ptip = np.array(ptip)

    #         # Position error
    #         e = p_des - ptip
    #         err_norm = np.linalg.norm(e)

    #         if err_norm < tol:
    #             break

    #         # Compute pseudo-inverse of Jv
    #         # TODO: change to weighted psuedo-inverse 
    #         # TODO: add repel away from centre sphere
    #         # If your manipulator is 3-DOF and you only care about x,y,z,
    #         # Jv is 3x3. We can invert or use np.linalg.pinv:
    #         Jv_pinv = np.linalg.pinv(Jv)  

    #         # Update rule
    #         dq = self.lam * (Jv_pinv @ e)
    #         q  = q + dq

    #     # Return final q
    #     return list(q)

    # def cartesian_to_joint_space_ik(self, x, y, z,
    #                                 q_init=None,
    #                                 tol=1e-3,
    #                                 max_iter=100,
    #                                 damping=0.01):
    #     """
    #     Numerical IK with Weighted Pseudo-inverse and Damping (Levenberg-Marquardt).
        
    #     p_des = [x, y, z]
    #     Weighted + damped inverse:
    #     Jv_pinv = W^-1 * Jv^T * (Jv * W^-1 * Jv^T + damping*I)^(-1)
    #     """
    #     p_des = np.array([x, y, z])

    #     # Start from q_init if provided, else from current
    #     if q_init is None:
    #         q = np.array(self.q_current, dtype=float)
    #     else:
    #         q = np.array(q_init, dtype=float)

    #     # Example weighting matrix (diagonal)
    #     W = np.diag([1.0, 2.0, 1.0])  # penalize 2nd joint more, etc.

    #     # Precompute W^-1
    #     W_inv = np.linalg.inv(W)

    #     for i in range(max_iter):
    #         # Forward kinematics to get tip position ptip
    #         ptip, Rtip, Jv, Jw = self.chain.fkin(q)
    #         ptip = np.array(ptip)

    #         # Position error
    #         e = p_des - ptip
    #         err_norm = np.linalg.norm(e)
    #         if err_norm < tol:
    #             break

    #         # Weighted + damped pseudo-inverse
    #         # Jv is size 3xN, W^-1 is NxN, so Jv * W_inv * Jv.T is 3x3.
    #         JWJt = Jv @ W_inv @ Jv.T
    #         # Add damping on the diagonal (3x3 identity).
    #         JWJt_damped = JWJt + damping * np.eye(JWJt.shape[0])
    #         JWJt_damped_inv = np.linalg.inv(JWJt_damped)

    #         # Weighted damped pseudo-inverse
    #         Jv_pinv_W_damped = W_inv @ Jv.T @ JWJt_damped_inv

    #         # Update rule with gain lam
    #         dq = self.lam * (Jv_pinv_W_damped @ e)

    #         # Update joint angles
    #         q = q + dq

    #     return list(q)

    def cartesian_to_joint_space_ik(self, x, y, z):
        # somehow make this to be called every update - this needs to be called every dt. 
        dt = 0.01
        q = np.array(self.q_current, dtype=float)
        ptip, Rtip, Jv, Jw = self.chain.fkin(q)
        J = Jv
        p = np.array(ptip)
        p_f = np.array([x, y, z])
        pd, vd = goto5(t, 2.0, p, p_f)
        e = p_f - ptip
        Jpinv = np.linalg.pinv(J)
        qddot = Jpinv @ (vd + self.lam * e)
        qd = self.q_current + qddot * dt
        self.x = ptip
        self.q_current = qd
        self.e = e

    # def cartesian_to_joint_space_ik(self, x, y, z, q_init=None, max_iter=100, tol=1e-4):
    #     """
    #     Iterative IK solver using the chain's forward kinematics (position only).
    #     x, y, z: desired end-effector position
    #     q_init:  initial guess (defaults to self.q_current)
    #     """
    #     if q_init is None:
    #         q_init = self.q_current

    #     # Convert to array
    #     q = np.array(q_init, dtype=float)
    #     p_des = np.array([x, y, z])

    #     # Step through a few iterations
    #     for i in range(max_iter):
    #         # Forward kinematics: returns (ptip, Rtip, Jv, Jw)
    #         ptip, Rtip, Jv, _ = self.chain.fkin(q)

    #         # Position error
    #         e = p_des - ptip
    #         norm_e = np.linalg.norm(e)

    #         if norm_e < tol:
    #             # Close enough
    #             break

    #         # 3xN Jacobian for linear velocity
    #         # In this 3-DOF example, Jv should be (3x3) if all joints are active
    #         # We can just invert if it's well-conditioned, or do a damped inverse:
    #         J = Jv  # shape (3,3)
    #         # If J is square and invertible:
    #         try:
    #             J_inv = np.linalg.inv(J)
    #         except np.linalg.LinAlgError:
    #             # fallback if singular: a damped least squares
    #             # J^\dagger = (J^T J + gamma^2 I)^{-1} J^T
    #             gamma = 0.01
    #             J_inv = np.linalg.inv(J.T @ J + gamma**2*np.eye(3)) @ J.T

    #         # Update rule: q_{n+1} = q_n + alpha * J_inv * e
    #         # We multiply the error by self.lam for faster or slower convergence
    #         alpha = 1.0  # step size
    #         qdot = alpha * self.lam * (J_inv @ e)
    #         q = q + qdot

    #     self.get_logger().info(
    #         f"IK finished in {i+1} iters, final error = {norm_e:.6f}")

    #     return q


    # def cartesian_to_joint_space_ik(self, x, y, z, q_init=None):
    #     """
    #     Simple iterative IK ignoring orientation.
    #       - p_des = [x, y, z]
    #       - Return final q once position error < tol or max_iter is reached.
    #     """
    #     p_des = np.array([x, y, z], dtype=float)
    #     q = np.array(self.q_current, dtype=float)
    #     q = np.array(q_init, dtype=float)

    #     for _ in range(self.max_iter):
    #         ptip, Rtip, Jv, Jw = self.chain.fkin(q)  
    #         ptip = np.array(ptip, dtype=float)

    #         # Position error
    #         error = p_des - ptip
    #         if np.linalg.norm(error) < self.tol:
    #             # Done!
    #             break
    #         Jv_pinv = np.linalg.pinv(Jv)

    #         dq = self.lam * (Jv_pinv @ error)  
    #         q += dq

    #     return q



    #
    # Move from current q to a new q over "duration" seconds via quintic splines
    #
    def move_with_spline(self, q_start, q_goal, duration):
        """Moves joints from q_start to q_goal over the given duration."""
        start_time = self.get_clock().now()

        while rclpy.ok():
            now     = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds * 1e-9
            t = min(elapsed, duration)

            positions, velocities = self.quintic_spline(q_start, q_goal, duration, t)
            self.sendcmd(positions, velocities)

            if t >= duration:
                self.sendcmd(positions, [0.0]*len(velocities))
                self.q_current = q_goal  
                break


    def move_to_waiting_position(self):
        duration = 2.0
        # intermediate then final:
        intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
        self.move_with_spline(self.q_current, intermediate_pos, duration)

        # waiting position
        waiting_position = [np.pi/2, 0.0, np.pi/2]
        self.move_with_spline(intermediate_pos, waiting_position, duration)

        self.get_logger().info("Reached waiting position.")

  
    def update(self):
        if self.current_phase == "startup":
            # Move from wherever we are to a "waiting" posture
            self.move_to_waiting_position()
            self.current_phase = "waiting"

        elif self.current_phase == "waiting":
            # Only move if there's a new point
            if self.point_queue:
                self.current_target = self.point_queue.pop(0)
                self.current_phase  = "moving"
            else:
                # No new target, do nothing
                pass

        elif self.current_phase == "moving":
            x, y, z = self.current_target
            try:
                q_target = self.cartesian_to_joint_space_ik(x, y, z)
            except np.linalg.LinAlgError:
                self.get_logger().info("Numerical error in IK. Skipping target.")
                self.current_phase = "waiting"
                return

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




