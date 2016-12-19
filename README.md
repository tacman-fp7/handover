# handover
Code and project activities to deal with Task 4.4.

## Final Goal

The final goal consists of making the robot able to transfer objects from one hand to the other. In particular:
- the robot is given an object in one of its hand (names as _first hand_)
- the object is localized inside the robot hand
- **in-hand object localization** allows to **select a good pose** for the other hand (_second hand_)
- the second arm reaches the selected pose, possibly by moving both the arms
- when the second hand achieves a stable grasp on the object, the first hand release it

## Modules

The modules are located under the `src` directory:
- `grasping-pose-selection` selects the best pose for the _second hand_
- `in-hand-localization` localizes the object in the _first hand_ by using vision and tactile information

## How to compile

Currently, the code for pose selection and for object segmentation has to be compiled separatly. An example for Linux system is following. 

In `grasping-pose-selection`, in `in-hand-localization/src/in-hand-segmentation` and `in-hand-localization/src/in-hand-localizer` directories:
```
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

### Running the modules

1. launch `pose-selection`, `in-hand-segmentation` and `in-hand-localizer` (the three binaries obtained after compilation)
2. make all the connections (see the [`handover.xml.template`](https://github.com/tacman-fp7/handover/blob/master/app/script/handover.xml.template) file)

You can play with the modules by using 3 rpc ports. Several commands are available,  but the most important ones are:

- `/in-hand-segmentation/rpc`:
      - `go_acquire` to acquire in-hand object point cloud
      - `go_filter` to filter it
- `/in-hand-localizer/rpc`:
      - `go_acquire` to get the  filtered object point cloud
      - `go_localize` to localize the object
- `/pose-selection/rpc`:
     - `ask_new_pose`: to get the estimated object pose and select a good pose for the _second hand_

What you can see on  yarpviews is the blob selected for point cloud extraction and the selected pose on the object.
Here, you can find an example of [the selected blob](https://github.com/tacman-fp7/handover/issues/17#issuecomment-267567631) and [the selected pose](https://github.com/tacman-fp7/handover/issues/15#issuecomment-265692371).





