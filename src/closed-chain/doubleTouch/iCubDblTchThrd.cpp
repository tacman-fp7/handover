#include "iCubDblTchThrd.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

doubleTouchThread::doubleTouchThread(int _rate, const string &_name, const string &_robot, int _v,
                                     double _jnt_vels,bool _dontgoback, const Vector &_hand_poss_master,
                                     const Vector &_hand_poss_slave, bool &go, bool &automatic_start) :
                                     RateThread(_rate), name(_name), robot(_robot),verbosity(_v),
                                     jnt_vels(_jnt_vels), dontgoback(_dontgoback), handPossMaster(_hand_poss_master),
                                     handPossSlave(_hand_poss_slave)
{
    step     = 0;

    armPossHome.resize(7,0.0);
    armPossHome[0]=-30.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHome[1]=80.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHome[3]=45.0*iCub::ctrl::CTRL_DEG2RAD;

    armPossHomeM.resize(7,0.0);
    armPossHomeM[0]=-30.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeM[1]=80.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeM[3]=45.0*iCub::ctrl::CTRL_DEG2RAD;

    armPossHomeS.resize(7,0.0);
    armPossHomeS[0]=-60.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeS[1]=24.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeS[2]=56.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeS[3]=40.0*iCub::ctrl::CTRL_DEG2RAD;
    armPossHomeS[4]=-50.0*iCub::ctrl::CTRL_DEG2RAD;

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

/************************************************************************/
bool doubleTouchThread::threadInit()
{
    portPoseIn.open("/"+name+"/ps:rpc");    
    portRpc.open("/"+name+"/rpc");

    attach(portRpc);

    Network::connect("/"+name+"/ps:rpc","/pose-selection/rpc");

    Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name +"/right_arm").c_str());
    if (!ddR.open(OptR))
    {
        yError(" Could not open right_arm PolyDriver!");
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
        yError(" Could not open left_arm PolyDriver!");
        return false;
    }

    bool ok = true;

    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(iposR);
        ok = ok && ddR.view(imodeR);
        ok = ok && ddR.view(iimpR);
        ok = ok && ddR.view(ilimR);
        ok = ok && ddR.view(crtlmodeR);
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
        ok = ok && ddR.view(crtlmodeL);
    }
    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    if (!ok)
    {
        yError(" Problems acquiring either left_arm or right_arm interfaces!!!!\n");
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
            yError(" Problems settings impedance values for either left_arm or right_arm!!!\n");
            return false;
        }
    }

    pos.resize(3,0.0);
    orient.resize(4,0.0);
    Hpose_waypoint.resize(4,4);
    Hpose.resize(4,4);
    H_hand.resize(4,4);

    askMovingArm();

    home=false;
    go=go_slave=go_master=false;


    //HIndex=receivePose("arm");

//    if (moving_arm=="right")
//        extractInitialQ(iencsR);
//    else if (moving_arm=="left")
//        extractInitialQ(iencsL);

    return true;
}

/************************************************************************/
void doubleTouchThread::run()
{
    if (checkMotionDone())
    {
        switch (step)
        {
            case 0:
                if (go)
                    step++;

                break;                
            case 1:
                if (askSelectedPose())
                    step++;

                break;
            case 2:
                //configureHands();
                goToPose();
                step++;

                break;
            case 3:

                Time::delay(3.0);
                testAchievement();
                step ++;

                break;
            case 4:

                if (!dontgoback)
                {
                    printf(" Going to rest...\n");
                    clearTask();
                    steerArmsHomeMasterSlave();
                    step=0;
                }
                step++;

                break;
            case 5:

                printf(" Switching to position mode..\n");
                imodeS -> setInteractionMode(2,VOCAB_IM_STIFF);
                imodeS -> setInteractionMode(3,VOCAB_IM_STIFF);
                step = 1;

                break;
            default:
                yError(" DoubleTouchThread should never be here!!!\nStep: %d",step);
                Time::delay(2.0);
                break;
        }
    }
}

/************************************************************************/
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
     crtlmodeM =   crtlmodeR;

        iencsS = iencsL;
        imodeS = imodeL;
         iposS =  iposL;
         encsS =  encsL;
         ilimS =  ilimL;
         jntsS =  jntsL;
          armS =   armL;
     crtlmodeS =   crtlmodeL;
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
     crtlmodeM =   crtlmodeL;


        iencsS = iencsR;
        imodeS = imodeR;
         iposS =  iposR;
         encsS =  encsR;
         ilimS =  ilimR;
         jntsS =  jntsR;
          armS =   armR;
     crtlmodeS =   crtlmodeR;
    }
    else
    {
        yError(" Current task type is none of the admissible values!");
        return false;
    }

    if (!alignJointsBounds())
    {
        yError(" AlignJointsBounds failed!!!\n");
        return false;
    }

    HIndex=eye(4);
    slv->probl->limb.setHN(HIndex);
    testLimb->setHN(HIndex);

    return true;
}

/************************************************************************/
bool doubleTouchThread::clearTask()
{
    cout<<" Clearing task.."<<endl;

    delete slv; slv=NULL;
    delete gue; gue=NULL;
    delete sol; sol=NULL;
    delete testLimb; testLimb=NULL;

    return true;
}

/************************************************************************/
void doubleTouchThread::configureHands()
{
    Vector vels(9,100.0);    
    vels[8]=200.0; 

    cout<<endl<<" Configuring master hand..."<<endl;

    for (int i=7; i<jntsM; i++)
    {
        iposM->setRefAcceleration(i,1e9);
        iposM->setRefSpeed(i,vels[i-7]);
        iposM->positionMove(i,handPossMaster[i-7]);
    }
}

/************************************************************************/
bool doubleTouchThread::checkMotionDone()
{
    if (step == 4 || step == 2 || step == 3)
        return true;

    if (home)
    {
        step=0;
        return true;
    }
    
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
    if (step >=2)
    {
        printf(" step: %i  result: %i  normL: %g\tnormR: %g\n", step,
            (normL <= VEL_THRES * getRate()) && (normR <= VEL_THRES * getRate()), normL, normR);
    }

    oldEEL = eeL;
    oldEER = eeR;

    if ((normL <= VEL_THRES * getRate()) && (normR <= VEL_THRES * getRate()))
    {
        return true;
    }

    return false;
}

/************************************************************************/
void doubleTouchThread::testAchievement()
{
    iencsM->getEncoders(encsM->data());
    iencsS->getEncoders(encsS->data());

    testLimb->setAng((*encsS)*iCub::ctrl::CTRL_DEG2RAD,(*encsM)*iCub::ctrl::CTRL_DEG2RAD);
    cout<<endl;
    printf(" Final end effector :          %s\n", testLimb->EndEffPosition().toString(3,3).c_str());
    printf(" Final end effector :          %s\n", testLimb->EndEffPose(true).toString(3,3).c_str());
    printf(" Final Joint configuration:    %s\n",(testLimb->getAng()*iCub::ctrl::CTRL_RAD2DEG).toString(3,3).c_str());
    cout<<endl;
}

/************************************************************************/
void doubleTouchThread::solveIK()
{
    slv->probl->limb.setH0(SE3inv(Hpose));
    testLimb->setH0(SE3inv(Hpose));

    slv->probl->limb.setAng(sol->joints);
    slv->setInitialGuess(*sol);
    slv->solve(*sol);
    solution=iCub::ctrl::CTRL_RAD2DEG * sol->joints;

    testLimb->setAng(sol->joints);
}

/************************************************************************/
void doubleTouchThread::goToPose()
{
    if (!home)
    {        
        if (go_slave)
        {
            goToPoseSlave();
        }

        Time::delay(2.0);

        if (go_master)
        {
            goToPoseMaster();
        }
    }
}

/************************************************************************/
void doubleTouchThread::goToPoseMaster()
{
    int nJnts = 7;
    Vector qM(nJnts,0.0);
    std::vector<int> Ejoints;

    for (size_t i=0; i<7;i++)
    {
        crtlmodeM->setControlMode(i,VOCAB_CM_POSITION);
    }

    if (verbosity>1)
    {
        cout<<" Moving master links: "<<endl;
    }
    for (int i = 0; i < 7; i++)
    {
        Ejoints.push_back(i);

            qM[i] = iCub::ctrl::CTRL_RAD2DEG*joints_sol[index][nDOF-7+i];

        if (verbosity>1)
        {
            printf(" #%i to: %g\t",i,qM[i]);
        }
    }
    if (verbosity>1)
    {
        cout<<endl;
    }

    iposM -> positionMove(nJnts,Ejoints.data(),qM.data());
}

/************************************************************************/
void doubleTouchThread::goToPoseSlave()
{
    cout<< "debug "<<endl;

    for (size_t i=0; i<7;i++)
    {
        cout<<" mode "<<crtlmodeS->setControlMode(i,VOCAB_CM_POSITION)<<endl;
    }

    if (verbosity>1)
    {
        cout<<" Moving slave  links: "<<endl;
    }
    for (int i = 0; i < nDOF-7; i++)
    {
        /*
        if (verbosity>1)
        {
            printf(" #%i to: %g\t",nDOF-7-1-i,-solution[i]);
        }*/

        iposS -> positionMove(nDOF-7-1-i,-iCub::ctrl::CTRL_RAD2DEG*joints_sol[index][i]);

    }
    if (verbosity>1)
    {
        cout<<endl;
    }
}

/************************************************************************/
void doubleTouchThread::steerArmsHome()
{   
    printf(" Moving arms to home, i.e. %s...\n",
                 (iCub::ctrl::CTRL_RAD2DEG*armPossHome).toString(3,3).c_str());

    for (size_t i=0; i<7;i++)
    {
        cout<<" control mode L "<<crtlmodeL->setControlMode(i,VOCAB_CM_POSITION)<<endl;
    }


    for (size_t i=0; i<7;i++)
    {
        cout<<" control mode R "<<crtlmodeR->setControlMode(i,VOCAB_CM_POSITION)<<endl;
    }

    if ( moving_arm=="left")
    {
        for (int i = 0; i < 7; i++)
        {
            iposL->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
        }

        Time::delay(2.0);
    }
    else
    {
        for (int i = 0; i < 7; i++)
        {
            iposR->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHome[i]);
        }
    }
}

/************************************************************************/
void doubleTouchThread::steerArmsHomeMasterSlave()
{
    printf(" Moving master arm to home, i.e. %s...\n",
                 (iCub::ctrl::CTRL_RAD2DEG*armPossHomeM).toString(3,3).c_str());

    printf(" Moving slave arm to home, i.e. %s...\n",
                 (iCub::ctrl::CTRL_RAD2DEG*armPossHomeS).toString(3,3).c_str());

    for (size_t i=0; i<7;i++)
    {
        cout<<" control mode L "<<crtlmodeL->setControlMode(i,VOCAB_CM_POSITION)<<endl;
    }


    for (size_t i=0; i<7;i++)
    {
        cout<<" control mode r "<<crtlmodeR->setControlMode(i,VOCAB_CM_POSITION)<<endl;
    }

    for (int i = 0; i < 7; i++)
    {
        iposM->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHomeM[i]);
    }
//    for (int i = 7; i < 16; i++)
//    {
//        if (i==7)   iposM -> positionMove(i,60.0);
//        else        iposM -> positionMove(i,0.0);
//    }

    Time::delay(2.0);
    
    for (int i = 0; i < 7; i++)
    {
        iposS->positionMove(i,iCub::ctrl::CTRL_RAD2DEG*armPossHomeS[i]);
    }
//    for (int i = 7; i < 16; i++)
//    {
//        if (i==7)   iposS -> positionMove(i,60.0);
//        else        iposS -> positionMove(i,0.0);
//    }
}

/************************************************************************/
bool doubleTouchThread::alignJointsBounds()
{
    deque<IControlLimits*> lim;
    lim.push_back(ilimS);
    lim.push_back(ilimM);

    if (testLimb->       alignJointsBounds(lim) == 0) return false;
    if (slv->probl->limb.alignJointsBounds(lim) == 0) return false;

    lim.pop_front();

    return true;
}

/************************************************************************/
void doubleTouchThread::threadRelease()
{
    cout<<endl;
    printf(" Returning to position mode..\n");
//    if (!dontgoback)
//    {
        steerArmsHome();
        imodeL -> setInteractionMode(2,VOCAB_IM_STIFF);
        imodeL -> setInteractionMode(3,VOCAB_IM_STIFF);
        imodeR -> setInteractionMode(2,VOCAB_IM_STIFF);
        imodeR -> setInteractionMode(3,VOCAB_IM_STIFF);
        steerArmsHome();
//    }

    delete encsR; encsR = NULL;
    delete  armR;  armR = NULL;

    delete encsL; encsL = NULL;
    delete  armL;  armL = NULL;

    cout<<endl;
    printf(" Closing ports..\n");

    printf(" Closing controllers..\n");
    ddR.close();
    ddL.close();

    printf(" Closing solver..\n");
    clearTask();
    cout<<endl;
}

/************************************************************************/
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
    else
        yError()<<" No pose received!";

    H=axis2dcm(orient);
    H.setSubcol(pos, 0, 3);
    H(3,3)=1;

    return H;
}

/************************************************************************/
bool doubleTouchThread::askMovingArm()
{
    Bottle cmd, reply;
    cmd.addString("get_moving_arm");

    if (portPoseIn.write(cmd,reply))
    {
        moving_arm=reply.get(0).asString();
        return true;
    }
    else
    {
        yError()<<" Moving arm name not received!!";
        return false;
    }
}

/************************************************************************/
void doubleTouchThread::askHhand()
{
    Bottle cmd, reply;
    cmd.addString("get_Hhand");

    if (portPoseIn.write(cmd, reply))
    {
        Bottle *rec=reply.get(0).asList();
        H_hand(0,0)=rec->get(0).asDouble(); H_hand(0,1)=rec->get(1).asDouble(); H_hand(0,2)=rec->get(2).asDouble(); H_hand(0,3)=rec->get(3).asDouble();
        H_hand(1,0)=rec->get(4).asDouble(); H_hand(1,1)=rec->get(5).asDouble(); H_hand(1,2)=rec->get(6).asDouble(); H_hand(1,3)=rec->get(7).asDouble();
        H_hand(2,0)=rec->get(8).asDouble(); H_hand(2,1)=rec->get(9).asDouble(); H_hand(2,2)=rec->get(10).asDouble(); H_hand(2,3)=rec->get(11).asDouble();
    }

    H_hand(3,3)=1.0;
}

/************************************************************************/
bool doubleTouchThread::askSelectedPose()
{
    Bottle cmd, reply;
    cmd.addString("get_index");

    if (portPoseIn.write(cmd, reply))
    {
        if (reply.get(0).asInt()<1000)
        {
            index=reply.get(0).asInt();
        }
        else
            return false;
    }
    else
        return false;

    return true;
}

/************************************************************************/
void doubleTouchThread::computeManip()
{
    pos_in_hand.clear();
    orie_in_hand.clear();
    manip.clear();
    Matrix aux(4,4);
    aux.zero();
    aux(2,0)=aux(1,2)=-1.0;
    aux(0,1)=1.0;

    askHhand();

    for (size_t i=0; i<positions.size(); i++)
    {
        Vector tmp(4,1.0);
        Vector tmp_pos(3,0.0);

        tmp.setSubvector(0, positions[i]);
        tmp=SE3inv(H_hand)*tmp;
        tmp_pos=tmp.subVector(0,2);
        pos_in_hand.push_back(tmp_pos);

        tmp=dcm2axis(aux*SE3inv(H_hand)*axis2dcm(orientations[i]));
        orie_in_hand.push_back(tmp);
    }

    cout<<endl;

    cout<<endl<<" Computing solutions..."<<endl<<endl;

    for (size_t i=0; i<positions.size(); i++)
    {
        Hpose=axis2dcm(orie_in_hand[i]);
        Hpose.setSubcol(pos_in_hand[i], 0, 3);
        if (askMovingArm())
        {            
            selectTask();
            solveIK();

            joints_sol.push_back(sol->joints);

            Matrix J=slv->probl->limb.asChainMod()->GeoJacobian(joints_sol[i]);

            manip.addDouble(sqrt(det(J*J.transposed())));
        }
        else
        {
            yError()<<" Moving arm name missing! Manipulability cannot be computed...";
            cout<<endl;
        }
    }

    cout<<endl<<" Solutions computed: waiting for moving.."<<endl<<endl;
}

/************************************************************************/
bool doubleTouchThread::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/************************************************************************/
Bottle doubleTouchThread::compute_manipulability(const Bottle &entry)
{
    Bottle *lstpos=entry.get(0).asList();
    Bottle *lstorie=entry.get(1).asList();

    Vector tmp(3,0.0);
    Vector tmp_o(4,0.0);

    positions.clear();
    orientations.clear();

    cout<<endl;

    if (lstpos->get(0).asString()=="positions")
    {
        for (size_t i=1; i<lstpos->size();i++)
        {
            Bottle *pos=lstpos->get(i).asList();
            tmp[0]=pos->get(0).asDouble();
            tmp[1]=pos->get(1).asDouble();
            tmp[2]=pos->get(2).asDouble();
            positions.push_back(tmp);
        }        
    }
    else
        yError()<<" No positions received!!";

    if (lstorie->get(0).asString()=="orientations")
    {
        for (size_t i=1; i<lstorie->size();i++)
        {
            Bottle *ori=lstorie->get(i).asList();
            tmp_o[0]=ori->get(0).asDouble();
            tmp_o[1]=ori->get(1).asDouble();
            tmp_o[2]=ori->get(2).asDouble();
            tmp_o[3]=ori->get(3).asDouble();

            orientations.push_back(tmp_o);
        }
    }
    else
        yError()<<" No orientaions received!!";

    computeManip();

    if (automatic_start)
        ;

    return manip;
}

/************************************************************************/
Bottle doubleTouchThread::get_solutions()
{
    Bottle reply;

    cout<<endl;
    for (size_t i=0; i<joints_sol.size();i++)
    {
        Bottle &cont=reply.addList();

        for (size_t j=0; j<joints_sol[i].size(); j++)
        {
            cont.addDouble(joints_sol[i][j]);
        }
    }

    return reply;
}

/************************************************************************/
bool doubleTouchThread::go_home()
{
    printf(" Going to rest...\n");
    go=false;
    home=true;

    steerArmsHomeMasterSlave();

    return true;
}

/************************************************************************/
bool doubleTouchThread::move(const string &entry)
{
    home=false;
    if (entry=="first_hand")
    {
        go_slave=true;
        go_master=false;
    }
    else if (entry=="second_hand")
    {
        go_master=true;
        go_slave=false;
    }
    else if (entry=="both")
        go_slave=go_master=true;
    go=true;

    return true;
}

/************************************************************************/
void doubleTouchThread::extractInitialQ(IEncoders *iencs)
{
    double v;
//    for (size_t i=0; i<7; i++)
//    {
//        iencs->getEncoder(i,&v);
//        armPossHomeS[i]=v;
//        cout<<armPossHomeS[i]<<endl;
//    }

    //iencsR->getEncoders(armPossHomeS.data());
    //cout<<"test "<<armL->getAng().toString()<<endl;
    armPossHomeS=armL->getAng();
    //cout<<"armPossHomeS "<<armPossHomeS.toString()<<endl;
}





