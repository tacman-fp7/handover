# handover
Code and project activities to deal with Task 4.4.

## Final Goal

The final goal consists of making the robot able to transfer objects from one hand to the other. In particular:
- the robot is given an object in one of its hand (names as _first hand_) (1)
- the object is localized inside the robot hand (2-3)
- **in-hand object localization** allows to **select a good pose** for the other hand (_second hand_) (4)
- the **second arm reaches** the selected pose, possibly by moving both the arms (5)
- when the second hand achieves a **stable grasp** on the object, the first hand release it (6)

<img src="https://github.com/tacman-fp7/handover/blob/devel/wiki/handover-pipeline3.png">

## Modules

The modules are located under the `src` directory:
- `grasping-pose-selection` selects the best pose for the _second hand_, communicating with `closed-chain module`
- `in-hand-localization` localizes the object in the _first hand_ by using **vision** and **tactile** information
- `closed-chain` provides a **two-arms kinematic chain** which considers the robot arms as a unique kinematics chain 

## How to compile

### Prerequisities

- [YARP](http://www.yarp.it/install.html)
- [icub-main and icub-contrib](http://wiki.icub.org/wiki/ICub_Software_Installation)
- [CGAL 4.7](https://github.com/CGAL/cgal)

### Compiling the code

 An example of compilation  for Linux system is the following:

```
 git clone https://github.com/tacman-fp7/handover.git
 cd handover
 mkdir build
 cd build
 ccmake ..
 make install
```
If you prefer instead compiling _separately_ each module:

```
cd handover/src/<moule_name>
mkdir build
cd build
ccmake ..
make install
```


## How to run 

The developed code can be run both on the **iCub simulator** and on the **real robot**.

### Prerequisities

1. [`CalibCameras`](http://wiki.icub.org/brain/group__icub__camCalib.html)
2. `iCubStartup` (in particular [`iKinCartesianSolver`](http://wiki.icub.org/brain/group__iKinCartesianSolver.html) for both arms,[`iKinGazeCtrl`](http://eris.liralab.it/iCub/main/dox/html/group__iKinGazeCtrl.html) )
3. [`SFM`](https://github.com/robotology/stereo-vision/tree/master/modules/SFM) and [`dispBlobber`](https://github.com/robotology/segmentation/tree/gh-pages)
4. [`stableGrasp`](https://github.com/tacman-fp7/tactile-control#how-to-run) for the **left** and the **right** hand

### Running the modules

4. launch `pose-selection`, `in-hand-segmentation`,`localizer` and `closed-chain` binaries with suitable configuration files
5. launch the `yarpviews` necessary for blob and pose selection visualization
6. make all the connections (see the [`handover_RTL.xml.template`](https://github.com/tacman-fp7/handover/blob/master/app/script/handover_RTL.xml.template) file as example)

(You can execute step `Prerequisities:3-4` and `Running the modules:1-3` just by launching and connecting port with the `handover` application in the `yarpmanager`)


#### Interactive mode

You can play with the modules by using 4 rpc ports. Several commands are available,  but the most important ones are:

- `/in-hand-segmentation/rpc`:
     - `go_acquire` to acquire in-hand object point cloud
     - `go_filter` to filter it
- `/in-hand-localizer/rpc`:
     - `go_acquire` to get the  filtered object point cloud
     - `go_localize` to localize the object
- `/pose-selection/rpc`:
     - `ask_new_pose`: to get the estimated object pose and select a good pose for the _second hand_
     -  `set_waypoint <no_waypoint>`: to make the second hand reach a specific waypoint
     - `reach_final`: to make the second hand reach the final pose
- `/closed-chain/rpc`:
     - `move_first_hand`: to make the robot move the first hand toward the final pose
  
#### Automatic  mode (Demo mode)
Alternatively, you can launch the [`lua script`](https://github.com/tacman-fp7/handover/blob/master/app/lua/handover_main_RTL.lua) that automatically executes all the `rpc` commands.
You just need to:
- open a port named: **/handover/go_on** 
- **launch and connect** all the modules
- **answer** to the question the lua script will ask you. In particular, the lua script stops 2 times. First, after the pose for the second hand is selected, the script will ask if you want to go on (typing **go_on**), to stop the module (**stop**) or to compute again the pose (**again**). Second, after the hands are in the final position, the lua will ask if to close the second hand or not. Then, you can answer: **go_on** if you want to perform the handover or **stop** if you want to stop the demo and the robot to go back to the home position.

Some examples of successful handovers are available here: [<img src="https://github.com/tacman-fp7/handover/blob/devel/wiki/play-video.png">](https://www.youtube.com/watch?v=be27-FGU-Sk).

[![DOI](https://zenodo.org/badge/70046565.svg)](https://zenodo.org/badge/latestdoi/70046565)







