# Standing and Balancing Controller for MIT Mini Cheetah  
Author:Junqi Ren(Karl)   
Environment:Ubuntu 14.04 /Qt 5.10.1  
This package contains two controllers written in C++,i.e. the standing controller and balancing controller. Both controllers work for the mini Cheetah from MIT in Qt environment. The controllers should also work well in any Ubuntu but unfortunately I cannot successfully build the MIT Cheetah package in Ubuntu 16.04. In that case, I run the simulation and execute the controllers in 14.04 version.  
In the standing controller, I use Bezier Cureve to design the trajectory for the locomotion. And as shown in the balancing controller which demands a time-varying ground force to keep balancing, I use qpoases as the optimization tool to achieve the very ground force, satisfying a series of dynamic inequalities.  
# Despendencies  
Check https://github.com/mit-biomimetics/Cheetah-Software . MIT Cheetah is a fascinating but complicated biomemetic robot program.  So, make sure you install every dependencies and build the whole package successfully.  
# Activate the Cheetah in Qt  
Download the two controller files and the new Cmakelist to `~/Cheetah-Software-Master/user`  
* Run the standing controller in Qt  
`./sim/sim`  
`./user/stand/stand_ctrl m s` where m "stands" for mini cheetah and "s" stands for simulation.  
* Run the  balancing controller
Similarly, open a new terminal and run the controller  
`./sim/sim`  
`./user/balance 1.05/balance_ctrl m s` 
