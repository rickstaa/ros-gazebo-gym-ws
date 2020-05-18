# Calculate stabilization torques

Here the maximum effort for the panda joints are calculated.

# Joint 2

## Solo
mass = 2.36414
Length = 1
Torque = m*g*(Length/2)
Torque = 2.36414*9.81*0.5
Torque = 11.5961067 Nm

## More joints
mass_up_chain = 2.36414+2.38050+2.42754+3.49611+1.46736+0.45606 = 12.59171
Length_up_chain = 6
Torque=12.59*9.81*3=370.52 Nm