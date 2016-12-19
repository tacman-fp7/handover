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

In `grasping-pose-selection` and in `in-hand-localization` directories:
```
 mkdir build
 cd build
 ccmake ..
 make install
```

## How to run 

A description on how to play with the two modules is provided at the following links:

- [**`grasping-pose-selection`**](https://github.com/tacman-fp7/handover/tree/master/src/grasping-pose-selection/app)
- [**`in-hand-localization`**](https://github.com/tacman-fp7/handover/tree/master/src/in-hand-localization/in-hand-segmentation/app)



