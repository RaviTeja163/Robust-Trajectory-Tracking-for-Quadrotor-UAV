# Robust-Trajectory-Tracking-for-Quadrotor-UAV
Generated a quintic trajectory and designed a robust inverse dynamics control law to enable a quadrotor to track it.

Youtube video of simulation result: https://youtu.be/brhxS1NvmZI

Documentation: https://github.com/RaviTeja163/Robust-Trajectory-Tracking-for-Quadrotor-UAV/blob/main/Report.pdf

The objective of this project is to develop a robust control scheme to enable a quadrotor to track desired trajectories in the presence of external disturbances. The control design under study will be tested on the Crazyflie 2.0 platform. Crazyflie is a quadrotor that is classified as a micro air vehicle (MAV), as it only weighs 27 grams and can fit in your hand. The size makes it ideal for flying inside a lab without trashing half the interior. Even though the propellers spin at high RPMs, they are soft and the torque in the motors is very low when compared to a brushless motor, making it relatively crash tolerant. The Crazyflie 2.0 features four 7mm coreless DC-motors that give the Crazyflie a maximum takeoff weight of 42g. The Crazyflie 2.0 is an open source project, with source code and hardware design both documented and available. For more information, please see the link below:
https://www.bitcraze.io/products/old-products/crazyflie-2-0/

Crazyflie 2.0 quadrotor needs to be set up in Gazebo by installing its ros dpendencies

'plot_trajectory.py' is a python script that generates quintic (fifth-order) trajectories (position, velocity and acceleration) for the translational coordinates (x, y, z) of Crazyflie. The quadrotor is starts from the origin p0 = (0, 0, 0) and visit five waypoints in sequence. The waypoints are
• p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
• p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
• p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
• p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
• p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 second

Considering the equations of motion of the quadrotor, we designed boundary layer-based sliding mode control laws for the z, ϕ, θ, ψ coordinates of the quadrotor to track desired trajectories.

'drone_smc_control.py'  implements a ROS node in Python to evaluate the performance of the control design on the Crazyflie 2.0 quadrotor in Gazebo.

Once the program is shut down, the actual trajectory is saved into a log.pkl file under the 'scripts' directory. A separate Python script 'visualize.py' helps visualize the trajectory from the saved log.pkl file.
