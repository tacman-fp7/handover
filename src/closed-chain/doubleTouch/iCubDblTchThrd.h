
#ifndef __DOUBLETOUCHTHREAD_H__
#define __DOUBLETOUCHTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include <iCub/periPersonalSpace/iCubDblTchSlv.h>
#include <iCub/periPersonalSpace/utils.h>


using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;

using namespace std;

class doubleTouchThread: public RateThread
{
protected:
    int verbosity;
    string name;
    string robot;
    string type;
    string curTaskType;
    int record;
    string filename;
    string moving_arm;
    string color;
    bool dontgoback;
    Vector handPossMaster; //hand configuration for "master" arm - 9 joints
    Vector handPossSlave; //hand configuration for "slave" arm - 9 joints

    Vector pos, orient;
    Matrix Hpose;

    int        step; // Flag to know in which step the thread is
    bool    recFlag; // Flag to know if the recording module has to record
    int        iter; // Iterator to keep track of the recording steps
    double jnt_vels; // Joint velocities during the double touch

    PolyDriver       ddR; // right arm device driver
    PolyDriver       ddL; // left arm  device driver
    PolyDriver       ddG; // gaze controller  driver

    // "Classical" interfaces - SLAVE ARM
    IEncoders         *iencsL;
    IPositionControl2 *iposL;
    IInteractionMode  *imodeL;
    IImpedanceControl *iimpL;
    IControlLimits    *ilimL;
    Vector            *encsL;
    iCubArm           *armL;
    int jntsL;
        
    // "Classical" interfaces - MASTER ARM
    IEncoders         *iencsR;
    IPositionControl2 *iposR;
    IImpedanceControl *iimpR;
    IInteractionMode  *imodeR;
    IControlLimits    *ilimR;
    Vector            *encsR;
    iCubArm           *armR;
    int jntsR;

    // "Classical" interfaces - SLAVE ARM
    IEncoders         *iencsS;
    IPositionControl2 *iposS;
    IInteractionMode  *imodeS;
    IImpedanceControl *iimpS;
    IControlLimits    *ilimS;
    Vector            *encsS;
    iCubArm           *armS;
    int jntsS;
        
    // "Classical" interfaces - MASTER ARM
    IEncoders         *iencsM;
    IPositionControl2 *iposM;
    IImpedanceControl *iimpM;
    IInteractionMode  *imodeM;
    IControlLimits    *ilimM;
    Vector            *encsM;
    iCubArm           *armM;
    int jntsM;

    doubleTouch_Variables *gue;
    doubleTouch_Variables *sol;
    doubleTouch_Solver    *slv;
    Vector solution;
    int nDOF;


    Vector armPossHome;

    // CUSTOM LIMB (for testing the achievement of the task)
    iCubCustomLimb *testLimb;

    // CHECKMOTIONDONE VARIABLES:
    Vector oldEEL;
    Vector oldEER;

    RpcClient portPoseIn;

    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Checks if the motion has finished. To be implemented in future releases
    * (the old version is not available any more because of the changes)
    */
    bool checkMotionDone();

    /**
    * Moves arms to starting (home) position
    */
    void steerArmsHome();
    void steerArmsHomeMasterSlave();

    /**
    * Configure hands to the pointing configuration
    */
    void configureHands();

    bool selectTask();
    bool clearTask();

    /**
    * Solves the Inverse Kinematic task
    */
    void solveIK();

    /**
    * Goes to the configuration found by solveIK()
    */
    void goToPose();
    void goToPoseMaster();
    void goToPoseSlave();

    /**
    * Sends the output to the port
    */
    void sendOutput();

    /**
    * Find the final configuration for the gaze interface to look at.
    */
    Vector findFinalConfiguration();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
    * Verify the degree of achievements of the task, by reading joints encoders
    */
    void testAchievement();

    void askMovingArm();

    void receivePose();

public:
    doubleTouchThread(int _rate, const string &_name, const string &_robot,
                      int _v, double _jnt_vels,
                      int _record, string _filename, string _color,
                      bool _dontgoback, const Vector &_hand_poss_master, const Vector &_hand_poss_slave);

    virtual bool threadInit();

    virtual void run();

    virtual void threadRelease();
};

#endif

