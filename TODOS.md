# TODOS

*   Rename start\_openai\_ros\_env module to make or REMOVE!.
*   Put negative reward for collision.
*   Add direct control.
*   Log level gripper command

# Open Upstream issues

*   Gazebo gravity compensation is wrong [see this issue](https://github.com/frankaemika/franka_ros/issues/160#issuecomment-961780423).
*   Franka simulated gripper does not have the right PID gains (https://github.com/frankaemika/franka\_ros/issues/172#issuecomment-979254189). I currently applied [8114ed28630851cd09eed7c92bac51a70ef1ee03](https://github.com/rickstaa/franka_ros/commit/8114ed28630851cd09eed7c92bac51a70ef1ee03) to [rickstaa/franka\_ros/commits/develop-panda\_gazebo](https://github.com/rickstaa/franka_ros/commits/develop-panda_gazebo) as a hotfix. The grasp action and thus the pick and place environment does not yet work with this hotfix.
