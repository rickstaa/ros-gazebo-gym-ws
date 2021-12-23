# TODOS

*   Rename start\_openai\_ros\_env module to make or REMOVE!.

*   Check step rate.

*   Profile code with austin to make it faster.

*   Gazebo gravity compensation not working see https://github.com/frankaemika/franka\_ros/issues/160

*   Compare goal and normal env.

# Open Upstream issues

*   Gazeo torque calculation is wrong (https://github.com/frankaemika/franka\_ros/issues/160#issuecomment-961780423).
*   Franka simulated gripper does not have the right PID gains (https://github.com/frankaemika/franka\_ros/issues/172#issuecomment-979254189). I currently applied [8114ed28630851cd09eed7c92bac51a70ef1ee03](https://github.com/rickstaa/franka_ros/commit/8114ed28630851cd09eed7c92bac51a70ef1ee03) to [rickstaa/franka\_ros/commits/develop-panda\_gazebo](https://github.com/rickstaa/franka_ros/commits/develop-panda_gazebo) as a hotfix. The grasp action and thus the pick and place environment does not yet work with this hotfix.
