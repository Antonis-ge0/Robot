# Robot
**Robot Movement in an Unknown Environment**

Implementation of a controller for a robot (SimBad) that will guide the robot through a set of obstacles toward its target, which will be defined by the projection of a light source (lamp) on the ground.

**The robot have at its disposal:**

*  12 distance sensors (max range: 1.5 m)

*  2 light sensors (in SimBad you can use either the getAverageLuminance() function or getLux() available in versions 1.7 and later – the use of the latter is recommended)

*  8 contact sensors

*  11 Infrared (IR) sensors (as many as you wish) mounted underneath the robot for line detection

**The robot does not:**

*  Know its position (no use of getCoords in SimBad)

*  Know the obstacles in the environment in advance (only local sensing via bumpers/sonars is being used)

*  Know the position of the target (the robot is being guided toward the target using its light sensors and IR sensors)

**Robot Objective:** The robot’s environment will include one lamp at a height of 2 m (this acts as the target). The robot stops moving as soon as it approaches the projection of the lamp on the ground at a distance equal to or less than 0.6 m. (Obviously, it was needed to conduct experiments to determine the brightness values collected by the sensors when they are this close to the lamp, in order to use them as a stopping condition.)

**Environment:**

The robot’s environment may include:

*  An unknown number of obstacles of various shapes and sizes

*  Lines on the floor of the environment. 

    Specifically, regarding the lines:
    
   *  Each obstacle may touch at least one line at one of its ends
    
   *  The lines may be placed either along the xx’ axis or the zz’ axis
    
   *  The lines will form various paths, one of which will lie along the shortest (according to Manhattan distance) path from the robot’s initial position to the lamp-target
    
     * This path may branch (forks – not crossroads) at various points toward directions that increase the Manhattan distance
    
     * This path may be interrupted (segments of the lines may be missing)
