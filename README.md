# Octobotics Coding Assignment

This Github repository provides a base inverted pendulum simulation to be used as-is for the purposes of this assignment


Send the assignment to: [ishan.b@octobotics.tech](ishan.b@octobotics.tech) 

![](src/inverted_pendulum_sim/media/inverted_pendulum_sim.png)



### Added package and how to run it

To run this clone the repository and then first run origianl code 
 ```bash
roslaunch inverted_pendulum_sim inverted_pendulum_sim.launch
```

After that in another terminal go to the repository and then run this command
```bash
rosrun inverted_controller control_force_publisher
```
