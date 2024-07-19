# ap_gazebo
This is a demo repository to open ArduCopter with SITL in gazebo.

Clone this repository and then source the environment and do the necessary changes as described in https://ardupilot.org/dev/docs/sitl-with-gazebo.html. 

After this use this command in terminal-1
```
gz sim -v4 -r iris_standoff.sdf
```

and this in terminal-2
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON  --console --map
```

The above two commands open iris with no extra environment in gazebo and runs SITL ardupilot respectively.

 - Run main.py file to see how to communciate with drone using pymavlink.
 - takeoff_land.py -> takes drone upto certain height and after reaching that height it returns to it's home location.
