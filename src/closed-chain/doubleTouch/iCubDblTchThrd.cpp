#include "iCubDblTchThrd.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

doubleTouchThread::doubleTouchThread(int _rate, const string &_name, const string &_robot, int _v,
                                     double _jnt_vels, int _record, string _filename, string _color,
                                     bool _autoconnect, bool _dontgoback, const Vector &_hand_poss_master, const Vector &_hand_poss_slave) :
                                     RateThread(_rate), name(_name), robot(_robot),verbosity(_v), skinParts(_skinParts), record(_record),
                                     filename(_filename), color(_color), autoconnect(_autoconnect), jnt_vels(_jnt_vels), dontgoback(_dontgoback),
                                     handPossMaster(_hand_poss_master),handPossSlave(_hand_poss_slave)
{
    step     = 0;
    recFlag  = 0;

    outPort  = new BufferedPort<Bottle>;

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
    outPort  -> open(("/"+name+"/status:o").c_str());

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

    return true;
}

void doubleTouchThread::run()
{
    if (checkMotionDone())
    {
        switch (step)
        {
            case 0:
                steerArmsHome();

                step++;
                break;

            case 1:
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

                if (curTaskType == "LHtoR" || curTaskType == "RHtoL")
                {
                    // to be changed
                    goToTaxelMaster();
                }
                else
                {
                    //to be changed
                    goToTaxelSlave();                    
                }
                
                step++;
                break;
            case 4:
                Time::delay(2.0);
                step++;
                break;
            case 5:
                if (curTaskType == "LHtoR" || curTaskType == "RHtoL")
                {
                    goToTaxelSlave();
                }
                else
                {
                    goToTaxelMaster();                    
                }
                step++;
                break;
            case 6:
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
            case 7:
                if(skinContacts)
                {
                    if (record == 1)
                    {
                        if (testAchievement2(skinContacts))
                            step++;
                        else if (robot == "icub" && exitFromDeadlock(skinContacts))
                            step++;
                    }
                    else if (record == 2)
                    {
                        if (exitFromDeadlock(skinContacts))
                            step++;
                        else
                            testAchievement2(skinContacts);
                    }
                }
                break;
            case 8:
                if (!dontgoback)
                {
                    printMessage(0,"Going to rest...\n");
                    clearTask();
                    steerArmsHomeMasterSlave();
                    step++;
                }
                break;
            case 9:
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
    switch(cntctSkinPart)
    {
        case SKIN_LEFT_HAND:
            curTaskType = "LHtoR";
            break;
        case SKIN_LEFT_FOREARM:
            curTaskType = "LtoR";
            break;
        case SKIN_RIGHT_HAND:
            curTaskType = "RHtoL";
            break;
        case SKIN_RIGHT_FOREARM:
            curTaskType = "RtoL";
            break;
    }

    slv = new doubleTouch_Solver(curTaskType);
    gue = new doubleTouch_Variables(slv->probl->getNVars()); // guess
    sol = new doubleTouch_Variables(slv->probl->getNVars()); // solution

    solution.resize(slv->probl->getNVars(),0.0);
    nDOF  = solution.size();

    if (curTaskType == "LtoR" || curTaskType == "RtoL")
    {
        gue->joints[1]   = -armPossHome[3]; gue->joints[3]   = -armPossHome[1];
        gue->joints[4]   = -armPossHome[0]; gue->joints[5]   = -armPossHome[0];
        gue->joints[6]   =  armPossHome[1]; gue->joints[8]   =  armPossHome[3];
    }
    else if (curTaskType == "LHtoR" || curTaskType == "RHtoL")
    {
        gue->joints[1+2] = -armPossHome[3]; gue->joints[3+2] = -armPossHome[1];
        gue->joints[4+2] = -armPossHome[0]; gue->joints[5+2] =  armPossHome[0];
        gue->joints[6+2] =  armPossHome[1]; gue->joints[8+2] =  armPossHome[3];
    }
    sol->clone(*gue);

    slv->probl->limb.setAng(gue->joints);

    testLimb = new iCubCustomLimb(curTaskType);

    if (curTaskType=="LHtoR" || curTaskType=="LtoR")
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
    else if (curTaskType=="RHtoL" || curTaskType=="RtoL")
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

    Vector joints;
    iencsM->getEncoders(encsM->data());
    slv->probl->index.getChainJoints(*encsM,joints);
    Matrix HIndex=slv->probl->index.getH(joints*iCub::ctrl::CTRL_DEG2RAD);
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

    // This has been made in order for sendOutput to work properly
    cntctSkinPart=SKIN_PART_UNKNOWN;

    return true;
}

void doubleTouchThread::configureHands()
{
    Vector vels(9,100.0);    
    vels[8]=200.0; 

    // Only one of them!!!
    printMessage(1,"Configuring master and slave hands...\n");
    for (int i=7; i<jntsM; i++)
    {
        iposM->setRefAcceleration(i,1e9);
        iposM->setRefSpeed(i,vels[i-7]);
        iposM->positionMove(i,handPossMaster[i-7]);
    }
    
    for (int i=7; i<jntsS; i++)
    {
        iposS->setRefAcceleration(i,1e9);
        iposS->setRefSpeed(i,vels[i-7]);
        iposS->positionMove(i,handPossSlave[i-7]);
    }
}

bool doubleTouchThread::exitFromDeadlock(skinContactList *_sCL)
{
    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if(it -> getSkinPart() == 7 && it -> getPressure() > 25)
        {
            // If the user touches the torso of the robot, it triggers
            // the return to the starting point
            printMessage(0,"I'm exiting from deadlock. Nothing is recorded! Iterator: %i\n",iter);
            return true;
        }
    }
    return false;
}

//bool doubleTouchThread::testAchievement2(skinContactList *_sCL)
//{
//    // Search for a suitable contact:
//    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
//    {
//        if(cntctSkinPart == it -> getSkinPart())
//        {
//            iencsS->getEncoders(encsS->data());
//            Vector qS=encsS->subVector(0,12);
//            armS->setAng(qS*iCub::ctrl::CTRL_DEG2RAD);

//            iencsM->getEncoders(encsM->data());
//            Vector qM=encsM->subVector(0,12);
//            armM->setAng(qM*iCub::ctrl::CTRL_DEG2RAD);

//            Matrix cntctH0_final = findH0(*it);

//            /**
//            * WRITE ON FILE: ITERATOR, ABSOLUTE TIME, YARP TIMESTAMP
//            * ROBOT, COLOR, TASKTYPE, ENCODERS SLAVE, ENCODERS MASTER,
//            * TARGET TAXEL REFERENCE FRAME, FINAL TAXEL REFERENCE FRAME,
//            * INDEX HN
//            */
//            printMessage(0,"SUCCESS!!!! Self Touch Accomplished! Iterator: %i\n",iter);
//            printMessage(1,"Encoders Slave:  %s\n", qS.toString(3,3).c_str());
//            printMessage(1,"Encoders Master: %s\n", qM.toString(3,3).c_str());
//            printMessage(0,"Target Position: %s\n", cntctH0.subcol(0,3,3).toString(3,3).c_str());
//            printMessage(0,"Final  Position: %s\n", cntctH0_final.subcol(0,3,3).toString(3,3).c_str());

//            ofstream outputfile;
//            outputfile.open (filename.c_str(),ios::app);
//            outputfile  << iter << "\t" << fixed << Time::now() << "\t"
//                        << robot  << "\t" << color << "\t" << curTaskType << "\t"
//                        << qS.toString(3,3) << "\t" << qM.toString(3,3) << "\t"
//                        << toVector(cntctH0).toString(3,3) << "\t"
//                        << toVector(cntctH0_final).toString(3,3);
//            outputfile  << "\t" << toVector(slv->probl->limb.getHN()).toString(3,3);
//            outputfile  << endl;
//            outputfile.close();
//            iter++;

//            return true;
//        }
//    }
//    return false;
//}

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
    printMessage(2,"Final Joint configuration:    %s\n",(testLimb->getAng()*iCub::ctrl::CTRL_RAD2DEG).toString(3,3).c_str());
}

void doubleTouchThread::solveIK()
{
    //HERE YOU HAVE TO CHANGE FOR SETTING YOUR DESIRED POSE
    printMessage(2,"H0: \n%s\n",pose.toString(3,3).c_str());
    slv->probl->limb.setH0(SE3inv(pose));
    testLimb->setH0(SE3inv(pose));

    slv->probl->limb.setAng(sol->joints);
    slv->setInitialGuess(*sol);
    slv->solve(*sol);
    // sol->print();
    solution=iCub::ctrl::CTRL_RAD2DEG * sol->joints;

    testLimb->setAng(sol->joints);
}

void doubleTouchThread::goToTaxel()
{
    goToTaxelSlave();
    Time::delay(2.0);
    goToTaxelMaster();
}

void doubleTouchThread::goToTaxelMaster()
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

    // devi impostare  affinche` sia palmo e non indice
    iposM -> positionMove(nJnts,Ejoints.data(),qM.data());
}

void doubleTouchThread::goToTaxelSlave()
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
    for (int i = 7; i < 16; i++)
    {
        if (i==7)   iposL -> positionMove(i,60.0);
        else        iposL -> positionMove(i,0.0);
    }

    Time::delay(2.0);
    
    for (int i = 0; i < 7; i++)
    {
        iposR->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
    }
    for (int i = 7; i < 16; i++)
    {
        if (i==7)   iposR -> positionMove(i,60.0);
        else        iposR -> positionMove(i,0.0);
    }
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
    if (slv->probl->index.alignJointsBounds(lim) == 0) return false;

    return true;
}

//bool doubleTouchThread::detectContact(skinContactList *_sCL)
//{
//    // Reset variables:
//    cntctPosLink.resize(3,0.0);
//    cntctPosWRF.resize(3,0.0);
//    cntctNormDir.resize(3,0.0);
//    cntctPressure = -1;
//    cntctLinkNum  = -1;
//    cntctSkinPart = SKIN_PART_UNKNOWN;

//    // Search for a suitable contact:
//    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
//    {
//        printMessage(3,"skinContact: %s\n",it->toString(3).c_str());
//        if( it -> getPressure() > 25 &&
//            norm(it-> getNormalDir()) != 0.0)
//        {
//            for (size_t i = 0; i < skinParts.size(); i++)
//            {
//                if (skinParts[i] == it -> getSkinPart())
//                {
//                    cntctSkin     = *it;                    // Store the skinContact for eventual future use
//                    cntctPosLink  = it -> getCoP();         // Get the position of the contact;
//                    cntctLinkNum  = it -> getLinkNumber();  // Retrieve the link number of the contact;
//                    cntctNormDir  = it -> getNormalDir();   // Normal direction of the contact
//                    cntctPressure = it -> getPressure();    // Retrieve the pressure of the contact

//                    cntctSkinPart = it -> getSkinPart();

//                    if (selectTask())
//                    {
//                        cntctPosWRF = locateContact();
//                        cntctH0     = findH0(cntctSkin);

/********************** NB ************************/
//                        //your contactHO IS THE POSE ON THE OBJECT
/********************** NB ************************/
//                        return true;
//                    }
//                }
//            }
//        }
//    }

//    return false;
//}

Vector doubleTouchThread::locateContact()
{
    Vector result(4,0.0);
    Matrix Twl = armS -> getH(cntctLinkNum+3, true);
    Vector posLink = cntctPosLink;
    posLink.push_back(1);
    result = Twl * posLink;
    result.pop_back();
    return result;
}

Matrix doubleTouchThread::findH0(skinContact &sc)
{
    // Set the proper orientation for the touching end-effector
    Matrix H0(4,4);
    Vector x(3,0.0), z(3,0.0), y(3,0.0);

    x = sc.getNormalDir();
    if (x[0] == 0.0)
    {
        x[0] = 0.00000001;    // Avoid the division by 0
    }

    if (curTaskType!="LHtoR" && curTaskType!="RHtoL")
    {
        z[0] = -x[2]/x[0]; z[2] = 1;
        y = -1*(cross(x,z));
    }
    else
    {
        // When x[0]==+-1, We can exploit an easier rule:
        z[1] = x[2];
        y = -1*(cross(x,z));
    }

    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);
    
    H0 = eye(4);
    H0.setSubcol(x,0,0);
    H0.setSubcol(y,0,1);
    H0.setSubcol(z,0,2);
    H0.setSubcol(sc.getCoP(),0,3);

    return H0;
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
        closePort(skinPort);
        printMessage(1,"skin port successfully closed!\n");
        closePort(outPort);
        printMessage(1,"output port successfully closed!\n");

    printMessage(0,"Closing controllers..\n");
        ddR.close();
        ddL.close();

    printMessage(0,"Closing solver..\n");
        clearTask();
}

// empty line to make gcc happy
