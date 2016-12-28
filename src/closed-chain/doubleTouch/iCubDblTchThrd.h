/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#ifndef __DOUBLETOUCHTHREAD_H__
#define __DOUBLETOUCHTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

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
using namespace iCub::skinDynLib;
using namespace iCub::iKin;

using namespace std;

class doubleTouchThread: public RateThread
{
protected:
    int verbosity;
    string name;
    string robot;
    // Type of the chain (either "LtoR", "RtoL", or "both")
    string type;
    string curTaskType;
    int record;
    string filename;
    // Color of the robot (to identify which one is which)
    string color;
    // Flag used to know if the doubleTouch should automatically connect to the skinManager
    bool autoconnect;
    // Flag used to know if the doubleTouch should go back after the first movement or not
    bool dontgoback;
    Vector handPossMaster; //hand configuration for "master" arm - 9 joints
    Vector handPossSlave; //hand configuration for "slave" arm - 9 joints

    // You must get rid of all these variables
    Vector cntctPosLink;    // Position in i-th link RF
    Vector cntctPosWRF;     // Position in WRF
    Vector cntctPosEE;      // Position in end-eff RF
    Vector cntctNormDir;    // Normal Direction
    Matrix cntctH0;         // RT matrix located in the contact with the 
                            // x axis normal to the cover
    int cntctLinkNum;       // Link number
    double cntctPressure;   // Pressure
    skinContact cntctSkin;  // SkinContact
    SkinPart cntctSkinPart;   // SkinPart (verbose form)


    int        step; // Flag to know in which step the thread is
    bool    recFlag; // Flag to know if the recording module has to record
    int        iter; // Iterator to keep track of the recording steps
    double jnt_vels; // Joint velocities during the double touch

    // You must get rid of all these variables
    std::vector<SkinPart> skinParts;
    BufferedPort<iCub::skinDynLib::skinContactList> *skinPort;
    BufferedPort<Bottle> *outPort;

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

    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Checks if the motion has finished. To be implemented in future releases
    * (the old version is not available any more because of the changes)
    */
    bool checkMotionDone();

    // Remove
    /**
    * Reads the contact from either /skinManager/skin_events:o or
    * /wholeBodyDynamics/contacts:o , and handles the skinContacts
    */
    bool detectContact(skinContactList *_sCL);

    /**
    * Finds the proper H0 for the limb
    * @param sc is the skinContact for which the H0 has to be computed
    */
    Matrix findH0(skinContact &sc);

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

    //REMOVE (taxel)
    /**
    * Goes to the configuration found by solveIK()
    */
    void goToTaxel();
    void goToTaxelMaster();
    void goToTaxelSlave();

    /**
    * Sends the output to the port
    */
    void sendOutput();

    // REMOVE
    /**
    * Locates the contact in World Reference Frame's coordinates
    */
    Vector locateContact();

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

    // REMOVE
    /**
    * Verify if the skin is touched. If so, it stores data.
    */
    bool testAchievement2(skinContactList *_sCL);

    /**
    * If testAchievement2 fails, we need a way to come back to the starting point!
    * This way will be a touch in the forearm :)
    */
    bool exitFromDeadlock(skinContactList *_sCL);

public:
    doubleTouchThread(int _rate, const string &_name, const string &_robot,
                      int _v, double _jnt_vels,
                      int _record, string _filename, string _color, bool _autoconnect,
                      bool _dontgoback, const Vector &_hand_poss_master, const Vector &_hand_poss_slave);

    virtual bool threadInit();

    virtual void run();

    virtual void threadRelease();
};

#endif

