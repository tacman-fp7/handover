#include "iCubDblTchThrd.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

doubleTouchThread::doubleTouchThread(int _rate, const string &_name, const string &_robot, int _v,
                                     double _jnt_vels, int _record, string _filename, string _color,
                                     bool _dontgoback, const Vector &_hand_poss_master, const Vector &_hand_poss_slave) :
                                     RateThread(_rate), name(_name), robot(_robot),verbosity(_v), record(_record),
                                     filename(_filename), color(_color), jnt_vels(_jnt_vels), dontgoback(_dontgoback),
                                     handPossMaster(_hand_poss_master),handPossSlave(_hand_poss_slave)
{
    step     = 0;
    recFlag  = 0;

    armPossHome.resize(7,0.0);
    armPossHome[0]=-30.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHome[1]=30.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHome[3]=45.0*iCub::ctrl::CTRL_DEG2RAD;

    armL = new iCubArm("left");
    armR = new iCubArm("right");

    iter = 1;

    oldEEL.resize(3,0.0);
    oldEER.resize(3,0.0);

    slv=NULL;
    gue=NULL;
    sol=NULL;
    testLimb=NULL;
}

bool doubleTouchThread::threadInit()
{
    portPoseIn.open("/"+name+"/ps:rpc");

    Network::connect("/"+name+"/ps:rpc","/pose-selection/rpc");

    Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name +"/right_arm").c_str());
    if (!ddR.open(OptR))
    {
        yError("[doubleTouch] Could not open right_arm PolyDriver!");
        return false;
    }

    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name +"/left_arm").c_str());
    if (!ddL.open(OptL))
    {
        yError("[doubleTouch] Could not open left_arm PolyDriver!");
        return false;
    }

    bool ok = 1;
    // Left arm is the master, right arm is the slave
    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(iposR);
        ok = ok && ddR.view(imodeR);
        ok = ok && ddR.view(iimpR);
        ok = ok && ddR.view(ilimR);
    }
    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    if (ddL.isValid())
    {
        ok = ok && ddL.view(iencsL);
        ok = ok && ddL.view(iposL);
        ok = ok && ddL.view(imodeL);
        ok = ok && ddL.view(iimpL);
        ok = ok && ddL.view(ilimL);
    }
    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    if (!ok)
    {
        yError("[doubleTouch] Problems acquiring either left_arm or right_arm interfaces!!!!\n");
        return false;
    }
    
    if (robot == "icub")
    {
        ok = 1;
        ok = ok && iimpL->setImpedance(0,  0.4, 0.03);
        ok = ok && iimpL->setImpedance(1, 0.35, 0.03);
        ok = ok && iimpL->setImpedance(2, 0.35, 0.03);
        ok = ok && iimpL->setImpedance(3,  0.2, 0.02);
        ok = ok && iimpL->setImpedance(4,  0.2, 0.00);
        
        ok = ok && iimpL->setImpedance(0,  0.4, 0.03);
        ok = ok && iimpL->setImpedance(1, 0.35, 0.03);
        ok = ok && iimpL->setImpedance(2, 0.35, 0.03);
        ok = ok && iimpL->setImpedance(3,  0.2, 0.02);
        ok = ok && iimpL->setImpedance(4,  0.2, 0.00);

        if (!ok)
        {
            yError("[doubleTouch] Problems settings impedance values for either left_arm or right_arm!!!\n");
            return false;
        }
    }

    pos.resize(3,0.0);
    orient.resize(4,0.0);
    Hpose.resize(4,4);

    askMovingArm();

    //HIndex=receivePose("arm");

    return true;
}

void doubleTouchThread::run()
{
    if (checkMotionDone())
    {
        switch (step)
        {
            case 0:
                step++;
                break;                
            case 1:
                Hpose=receivePose("pose");
                selectTask();
                step++;
                break;
            case 2:
                solveIK();
                yInfo("[doubleTouch] Going to pose... Desired EE: %s\n",(sol->ee).toString(3,3).c_str());
                printMessage(1,"Desired joint configuration:  %s\n",(sol->joints*iCub::ctrl::CTRL_RAD2DEG).toString(3,3).c_str());
                step++;
                recFlag = 1;
                break;
            case 3:
                configureHands();
                if (record != 0)
                {
                    Time::delay(2.0);
                }

                goToPose();
                
                step++;
                break;
            case 4:
                Time::delay(2.0);
                step++;
                break;
            case 5:
                recFlag = 0;
                
                bool flag;
                if (record == 0)
                {
                    Time::delay(3.0);
                    flag=1;
                    if (flag == 1)
                    {
                        testAchievement();
                        step += 2;
                    }
                }
                else
                {
                    testAchievement();
                    printMessage(0,"Waiting for the event to go back.\n");
                    step++;
                }
                break;
            case 6:
                if (!dontgoback)
                {
                    printMessage(0,"Going to rest...\n");
                    clearTask();
                    steerArmsHomeMasterSlave();
                    step++;
                }
                break;
            case 7:
                printMessage(1,"Switching to position mode..\n");
                imodeS -> setInteractionMode(2,VOCAB_IM_STIFF);
                imodeS -> setInteractionMode(3,VOCAB_IM_STIFF);
                yInfo("[doubleTouch] WAITING FOR CONTACT...\n");
                step = 1;
                break;
            default:
                yError("[doubleTouch] doubleTouchThread should never be here!!!\nStep: %d",step);
                Time::delay(2.0);
                break;
        }
    }
}

bool doubleTouchThread::selectTask()
{
    if (moving_arm=="right")
        curTaskType = "LHtoR";
    else if (moving_arm=="left")
        curTaskType = "RHtoL";


    slv = new doubleTouch_Solver(curTaskType);
    gue = new doubleTouch_Variables(slv->probl->getNVars()); // guess
    sol = new doubleTouch_Variables(slv->probl->getNVars()); // solution

    solution.resize(slv->probl->getNVars(),0.0);
    nDOF  = solution.size();

    gue->joints[1+2] = -armPossHome[3]; gue->joints[3+2] = -armPossHome[1];
    gue->joints[4+2] = -armPossHome[0]; gue->joints[5+2] =  armPossHome[0];
    gue->joints[6+2] =  armPossHome[1]; gue->joints[8+2] =  armPossHome[3];

    sol->clone(*gue);

    slv->probl->limb.setAng(gue->joints);

    testLimb = new iCubCustomLimb(curTaskType);

    if (curTaskType=="LHtoR")
    {
        iencsM = iencsR;
        imodeM = imodeR;
         iposM =  iposR;
         encsM =  encsR;
         ilimM =  ilimR;
         jntsM =  jntsR;
          armM =   armR;

        iencsS = iencsL;
        imodeS = imodeL;
         iposS =  iposL;
         encsS =  encsL;
         ilimS =  ilimL;
         jntsS =  jntsL;
          armS =   armL;
    }
    else if (curTaskType=="RHtoL")
    {
        iencsM = iencsL;
        imodeM = imodeL;
         iposM =  iposL;
         encsM =  encsL;
         ilimM =  ilimL;
         jntsM =  jntsL;
          armM =   armL;


        iencsS = iencsR;
        imodeS = imodeR;
         iposS =  iposR;
         encsS =  encsR;
         ilimS =  ilimR;
         jntsS =  jntsR;
          armS =   armR;        
    }
    else
    {
        yError("[doubleTouch] current task type is none of the admissible values!");
        return false;
    }

    if (!alignJointsBounds())
    {
        yError("[doubleTouch] alignJointsBounds failed!!!\n");
        return false;
    }

//    Vector joints;
//    iencsM->getEncoders(encsM->data());
//    slv->probl->index.getChainJoints(*encsM,joints);
//    HIndex=slv->probl->index.getH(joints*iCub::ctrl::CTRL_DEG2RAD);

    HIndex=eye(4);
    slv->probl->limb.setHN(HIndex);
    testLimb->setHN(HIndex);
    printMessage(1,"Index type: %s \t HIndex:\n%s\n", slv->probl->index.getType().c_str(),
                                                      HIndex.toString(3,3).c_str());

    return true;
}

bool doubleTouchThread::clearTask()
{
    yInfo("[doubleTouch] Clearing task..");

    delete slv; slv=NULL;
    delete gue; gue=NULL;
    delete sol; sol=NULL;
    delete testLimb; testLimb=NULL;

    return true;
}

void doubleTouchThread::configureHands()
{
    Vector vels(9,100.0);    
    vels[8]=200.0; 

    printMessage(1,"Configuring master hand...\n");

    for (int i=7; i<jntsM; i++)
    {
        iposM->setRefAcceleration(i,1e9);
        iposM->setRefSpeed(i,vels[i-7]);
        iposM->positionMove(i,handPossMaster[i-7]);
    }
}

bool doubleTouchThread::checkMotionDone()
{
    if (step == 7 || (record == 0 && (step == 4 || step == 5)))
        return true;
    
    iencsL->getEncoders(encsL->data());
    Vector qL=encsL->subVector(0,6);
    armL->setAng(qL*iCub::ctrl::CTRL_DEG2RAD);
    Vector eeL = armL -> EndEffPosition();

    iencsR->getEncoders(encsR->data());
    Vector qR=encsR->subVector(0,6);
    armR->setAng(qR*iCub::ctrl::CTRL_DEG2RAD);
    Vector eeR = armR -> EndEffPosition();

    double normL = norm(eeL - oldEEL);
    double normR = norm(eeR - oldEER);
    printMessage(4,"step: %i  result: %i  normL: %g\tnormR: %g\n", step,
        (normL <= VEL_THRES * getRate()) && (normR <= VEL_THRES * getRate()), normL, normR);

    oldEEL = eeL;
    oldEER = eeR;

    if ((normL <= VEL_THRES * getRate()) && (normR <= VEL_THRES * getRate())) {
        return true;
    }

    return false;
}

Vector doubleTouchThread::findFinalConfiguration()
{
    Vector q=solution.subVector(nDOF-1-7,nDOF-1);
    armM->setAng(q*iCub::ctrl::CTRL_DEG2RAD);
    return armM -> EndEffPosition();
}

void doubleTouchThread::testAchievement()
{
    iencsM->getEncoders(encsM->data());
    iencsS->getEncoders(encsS->data());

    testLimb->setAng((*encsS)*iCub::ctrl::CTRL_DEG2RAD,(*encsM)*iCub::ctrl::CTRL_DEG2RAD);
    printMessage(0,"Final end effector :          %s\n", testLimb->EndEffPosition().toString(3,3).c_str());
    printMessage(0,"Final end effector :          %s\n", testLimb->EndEffPose(true).toString(3,3).c_str());
    printMessage(2,"Final Joint configuration:    %s\n",(testLimb->getAng()*iCub::ctrl::CTRL_RAD2DEG).toString(3,3).c_str());
}

void doubleTouchThread::solveIK()
{
    printMessage(2,"H0: \n%s\n",Hpose.toString(3,3).c_str());
 //  slv->probl->limb.setH0(SE3inv(Hpose));
//   testLimb->setH0(SE3inv(Hpose));

    slv->probl->limb.setH0(Hpose);
    testLimb->setH0(Hpose);

    slv->probl->limb.setAng(sol->joints);
    slv->setInitialGuess(*sol);
    slv->solve(*sol);
    // sol->print();
    solution=iCub::ctrl::CTRL_RAD2DEG * sol->joints;

    testLimb->setAng(sol->joints);
}

void doubleTouchThread::goToPose()
{
    yDebug()<<"Moving slave ...";
    goToPoseSlave();
    Time::delay(2.0);
    yDebug()<<"Moving master ...";
    goToPoseMaster();
}

void doubleTouchThread::goToPoseMaster()
{
    int nJnts = 7;
    Vector qM(nJnts,0.0);
    std::vector<int> Ejoints;

    if (verbosity>1)
    {
        printf("[doubleTouch] Moving master links: ");
    }
    for (int i = 0; i < 7; i++)
    {
        Ejoints.push_back(i);
        qM[i] = solution[nDOF-7+i];
        if (verbosity>1)
        {
            printf("#%i to: %g\t",i,qM[i]);
        }
    }
    if (verbosity>1)
    {
        printf("\n");
    }

    iposM -> positionMove(nJnts,Ejoints.data(),qM.data());
}

void doubleTouchThread::goToPoseSlave()
{
    if (verbosity>1)
    {
        printf("[doubleTouch] Moving slave  links: ");
    }
    for (int i = 0; i < nDOF-7; i++)
    {
        if (verbosity>1)
        {
            printf("#%i to: %g\t",nDOF-7-1-i,-solution[i]);
        }
        iposS -> positionMove(nDOF-7-1-i,-solution[i]);
    }
    if (verbosity>1)
    {
        printf("\n");
    }
}

void doubleTouchThread::steerArmsHome()
{   
    printMessage(1,"Moving arms to home, i.e. %s...\n",
                 (iCub::ctrl::CTRL_RAD2DEG*armPossHome).toString(3,3).c_str());

    for (int i = 0; i < 7; i++)
    {
        iposL->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
    }
//    for (int i = 7; i < 16; i++)
//    {
//        if (i==7)   iposL -> positionMove(i,60.0);
//        else        iposL -> positionMove(i,0.0);
//    }

    Time::delay(2.0);
    
    for (int i = 0; i < 7; i++)
    {
        iposR->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
    }
//    for (int i = 7; i < 16; i++)
//    {
//        if (i==7)   iposR -> positionMove(i,60.0);
//        else        iposR -> positionMove(i,0.0);
//    }
}

void doubleTouchThread::steerArmsHomeMasterSlave()
{
    printMessage(1,"Moving arms to home, i.e. %s...\n",
                 (iCub::ctrl::CTRL_RAD2DEG*armPossHome).toString(3,3).c_str());

    for (int i = 0; i < 7; i++)
    {
        iposM->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
    }
    for (int i = 7; i < 16; i++)
    {
        if (i==7)   iposM -> positionMove(i,60.0);
        else        iposM -> positionMove(i,0.0);
    }

    Time::delay(2.0);
    
    for (int i = 0; i < 7; i++)
    {
        iposS->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
    }
    for (int i = 7; i < 16; i++)
    {
        if (i==7)   iposS -> positionMove(i,60.0);
        else        iposS -> positionMove(i,0.0);
    }
}

bool doubleTouchThread::alignJointsBounds()
{
    deque<IControlLimits*> lim;
    lim.push_back(ilimS);
    lim.push_back(ilimM);

    if (testLimb->       alignJointsBounds(lim) == 0) return false;
    if (slv->probl->limb.alignJointsBounds(lim) == 0) return false;

    lim.pop_front();
    //if (slv->probl->index.alignJointsBounds(lim) == 0) return false;

    return true;
}

int doubleTouchThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}

void doubleTouchThread::threadRelease()
{
    printMessage(0,"Returning to position mode..\n");
        if (!dontgoback)
        {
            steerArmsHome();
            imodeL -> setInteractionMode(2,VOCAB_IM_STIFF);
            imodeL -> setInteractionMode(3,VOCAB_IM_STIFF);
            imodeR -> setInteractionMode(2,VOCAB_IM_STIFF);
            imodeR -> setInteractionMode(3,VOCAB_IM_STIFF);
            steerArmsHome();
        }

        delete encsR; encsR = NULL;
        delete  armR;  armR = NULL;

        delete encsL; encsL = NULL;
        delete  armL;  armL = NULL;

    printMessage(0,"Closing ports..\n");

    printMessage(0,"Closing controllers..\n");
        ddR.close();
        ddL.close();

    printMessage(0,"Closing solver..\n");
        clearTask();
}

Matrix doubleTouchThread::receivePose(const string &what)
{
    Matrix H(4,4);
    Bottle cmd, reply;
    if (what=="pose")
        cmd.addString("get_pose");
    else if (what=="arm")
        cmd.addString("get_pose_moving_arm");

    if (portPoseIn.write(cmd,reply))
    {
        Bottle *rec=reply.get(0).asList();
        pos[0]=rec->get(0).asDouble();
        pos[1]=rec->get(1).asDouble();
        pos[2]=rec->get(2).asDouble();

        orient[0]=rec->get(3).asDouble();
        orient[1]=rec->get(4).asDouble();
        orient[2]=rec->get(5).asDouble();
        orient[3]=rec->get(6).asDouble();
    }

    yDebug()<<" Received pose: "<<pos.toString()<<" "<<orient.toString();

    H=axis2dcm(orient);
    H.setSubcol(pos, 0, 3);
    H(3,3)=1;

    cout<<endl<<"Hpose "<<Hpose.toString()<<endl<<endl;

    return H;
}

void doubleTouchThread::askMovingArm()
{
    Bottle cmd, reply;
    cmd.addString("get_moving_arm");

    if (portPoseIn.write(cmd,reply))
        moving_arm=reply.get(0).asString();
}

// empty line to make gcc happy
