# handover
Code and project activities to deal with Task 4.4.

## Final Goal

The final goal consists of making the robot able to transfer objects from one hand to the other. In particular:
- the robot is given an object in one of its hand (names as _first hand_)
- the object is localized inside the robot hand
- **in-hand object localization** allows to **select a good pose** for the other hand (_second hand_)
- the **second arm reaches** the selected pose, possibly by moving both the arms
- when the second hand achieves a **stable grasp** on the object, the first hand release it

## Modules

The modules are located under the `src` directory:
- `grasping-pose-selection` selects the best pose for the _second hand_, communicating with `closed-chain module`
- `in-hand-localization` localizes the object in the _first hand_ by using **vision** and **tactile** information
- `closed-chain` provides a **two-arms closed kinematic chain** which considers the robot arms as a unique kinematics chain 

## How to compile

### Prerequisities

- [YARP](http://www.yarp.it/install.html)
- [icub-main and icub-contrib](http://wiki.icub.org/wiki/ICub_Software_Installation)
- [CGAL 4.7](https://github.com/CGAL/cgal)

### Compiling the code

Currently, the `closed-chain` module has to be compiled separatly, since its structure is temporary. An example of compilation for `pose-selection` and `in-hand-localization` modules for Linux system is following:

```
 git clone https://github.com/tacman-fp7/handover.git
 cd handover
 mkdir build
 cd build
 ccmake ..
 make install
```
If you want to compile `closed-chain` module:

```
cd handover/src/closed-chain
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

### Running the modules

4. launch `pose-selection`, `in-hand-segmentation`,`in-hand-localizer` and `closed-chain` binaries with suitable configuration files
5. launch the two `yarpviews` necessary for blob and pose selection visualization
6. make all the connections (see the [`handover.xml.template`](https://github.com/tacman-fp7/handover/blob/master/app/script/handover.xml.template) file)

(You can execute step `Prerequisities:3` and `Running the modules:1-3` just by launching and connecting port with the `handover` application in the `yarpmanager`)

You can play with the modules by using 3 rpc ports. Several commands are available,  but the most important ones are:

- `/in-hand-segmentation/rpc`:
      - `go_acquire` to acquire in-hand object point cloud
      - `go_filter` to filter it
- `/in-hand-localizer/rpc`:
      - `go_acquire` to get the  filtered object point cloud
      - `go_localize` to localize the object
- `/pose-selection/rpc`:
     - `ask_new_pose`: to get the estimated object pose and select a good pose for the _second hand_

Yarpviews show the blob selected for point cloud extraction and the selected pose on the object.
Here, you can find an example of [the selected blob](https://github.com/tacman-fp7/handover/issues/17#issuecomment-267567631) and [the selected pose](https://github.com/tacman-fp7/handover/issues/15#issuecomment-265692371).
The best pose for the second arm can be chosen by using the standard kinematics or the closed-chain. As example of the second result is available [here](https://github.com/tacman-fp7/handover/issues/20#issuecomment-270431847).





