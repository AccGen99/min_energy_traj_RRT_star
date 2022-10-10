# min_energy_traj_RRT_star
Minimum Energy Trajectory Planning with RRT star algorithm

Implements the paper titled - Online minimum-energy trajectory planning and control on a straight-line path for three-wheeled omnidirectional mobile robots for planning velocity. Wayoints required for implementation are generated via RRT* algorithm in known environment in MATLAB

## File Description
derLamda.m - Derivative of lagragian multiplier constant for cost function calculation \
FindLambda.m - Calculate value of lagragian multipliers \
MERV.m - Solve differential equation of minimum energy rotational velocity for considered time-steps \
MERV_old.m - Previous trial (in-efficient) implementation. Don't use \
R_calc.m - Calculate rotation matrix of robot body at any arbitrary timestep \
R_dot_calc.m - Calculate derivative of rotation matrix for robot at any arbitrary timestep \
cVec.m - Calculate normalized control vector for all 3 motors \
calcAacc.m - Cleaner implementation of finding maximum possible acceleration for robot system with given parameters \
calcContVec.m - Cleaner implementation for calculating control vector \
checkIfReached.m - Manually set the distance from target when it is considered as 'reached' \
energyCalc.m - Calculate the cost function (energy drawn from batteries) \
maxAcc.m - Paper implementation of max acceleration calculation (Don't use) \
merv_ode.m - Defines ODE of rotational velocity \
merv_ode_calcs.m - Calculate constants of merv ODE \
min_energy.m - Finding value of tf satisfying constraints for minimum energy trajectory generation \
phiCalc.m - Calculate orientation of TOMR at given timestep \
robot_parameters.m - Define model parameters \
rrt_planner.m - RRT star planner \
run.m - Run minimum energy trajectory with RRT star \
trials.m - Check if minimum energy system is working \
vx_ode.m - Define ODE for translational velocity for numerical solving \
