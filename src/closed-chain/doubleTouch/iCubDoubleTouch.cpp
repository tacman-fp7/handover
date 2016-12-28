#include <yarp/os/Log.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>
 
#include <iostream>
#include <string.h> 
#include <ctime>
#include <sstream>

#include <iCub/periPersonalSpace/utils.h>
#include "iCubDblTchThrd.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

class doubleTouch: public RFModule
{
private:
    doubleTouchThread *doubleTouch_Thrd;
    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot;
    string name;
    string type;
    string filename;
    string color;

    int verbosity,rate,record;

    bool autoconnect, dontgoback;

    double jnt_vels;
    
    Vector handPossM; //hand configuration for "master" arm
    // I don't want to change the slave (it holds the object!)
    Vector handPossS; //hand configuration for "slave" arm

    std::vector<SkinPart> _sPs;

public:
    doubleTouch()
    {
        doubleTouch_Thrd=0;

        robot    = "icubSim";
        name     = "double-touch";
        type     = "LtoR";
        filename = ".txt";
        color    = " ";

        verbosity =    0;    // verbosity
        rate      =  100;    // rate of the doubleTouchThread
        record    =    0;    // record data
        jnt_vels  = 10.0;    // joint speed for the double touch

        autoconnect = false;
        dontgoback  = false;
        
        handPossM.resize(9,0.0);
        //default parameters correspond to master hand totally open
        handPossM[0]=40.0; handPossM[1]=0.0;
        handPossM[2]=0.0; handPossM[3]=0.0;
        handPossM[4]=0.0;  handPossM[5]=0.0;
        handPossM[6]=0.0; handPossM[7]=0.0;
        handPossM[8]=0.0;
        
        // I don't want to change fingers positions
        handPossS.resize(9,0.0);
        handPossS[0]=40.0;  handPossS[1]=10.0;  
        handPossS[2]=60.0;  handPossS[3]=70.0;  
        handPossS[4]=0.0;   handPossS[5]=0.0;
        handPossS[6]=0.0;   handPossS[7]=0.0;
        handPossS[8]=0.0;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('c','o','n','n'):
                {
                    Network yarpNetwork;
                    if (yarpNetwork.connect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str()))
                        reply.addVocab(ack);
                    else
                        reply.addVocab(nack);
                    return true;
                }

                case VOCAB4('s','t','a','r'):
                {
                    doubleTouch_Thrd = new doubleTouchThread(rate, name, robot, verbosity, _sPs,
                                                       jnt_vels, record, filename, color,
                                                       autoconnect, dontgoback, handPossM, handPossS);
                    bool strt = doubleTouch_Thrd -> start();
                    if (!strt)
                    {
                        delete doubleTouch_Thrd;
                        doubleTouch_Thrd = 0;
                        yError("doubleTouchThread wasn't instantiated!!");
                        reply.addVocab(nack);
                    }
                    else
                        reply.addVocab(ack);
                    return true;
                }

                case VOCAB4('d','i','s','c'):
                {
                    Network yarpNetwork;
                    if (yarpNetwork.disconnect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str()))
                        reply.addVocab(ack);
                    else
                        reply.addVocab(nack);
                    return true;
                }

                case VOCAB4('s','t','o','p'):
                {
                    if (doubleTouch_Thrd)
                    {
                        yInfo("DOUBLE TOUCH: Stopping threads..");
                        doubleTouch_Thrd->stop();
                        delete doubleTouch_Thrd;
                        doubleTouch_Thrd=0;
                    }
                    reply.addVocab(ack);
                    return true;
                }

                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    /*********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool alignEyes = rf.check("alignEyes");
        autoconnect    = rf.check("autoconnect");
        dontgoback     = rf.check("dontgoback");

        if (dontgoback)
        {
            yInfo("[doubleTouch] Dontgoback flag set to ON");
        }

        if (autoconnect)
        {
            yInfo("[doubleTouch] Autoconnect flag set to ON");
        }

        if (rf.check("name"))
        {
            name = rf.find("name").asString();
            yInfo("[doubleTouch] Module name set to %s", name.c_str());
        }
        else
            yInfo("[doubleTouch] Module name set to default, i.e. %s", name.c_str());

        setName(name.c_str());

        robot = rf.check("robot", Value("icubSim")).asString();
        yInfo("[doubleTouch] Robot is: %s", robot.c_str());

        type = rf.check("type", Value("RtoL")).asString();
        yInfo("[doubleTouch] Type is: %s", type.c_str());

        if (type=="RtoL")
        {
            _sPs.push_back(SKIN_RIGHT_FOREARM);
        }
        else if (type=="RHtoL")
        {
            _sPs.push_back(SKIN_RIGHT_HAND);
        }
        else if (type=="LtoR")
        {
            _sPs.push_back(SKIN_LEFT_FOREARM);
        }
        else if (type=="LHtoR")
        {
            _sPs.push_back(SKIN_LEFT_HAND);
        }
        else if (type=="all_RtoL")
        {
            _sPs.push_back(SKIN_RIGHT_FOREARM);
            _sPs.push_back(SKIN_RIGHT_HAND);
        }
        else if (type=="all_LtoR")
        {
            _sPs.push_back(SKIN_LEFT_FOREARM);
            _sPs.push_back(SKIN_LEFT_HAND);
        }
        else if (type=="all_12DoF")
        {
            _sPs.push_back(SKIN_LEFT_FOREARM);
            _sPs.push_back(SKIN_RIGHT_FOREARM);
        }
        else if (type=="all_14DoF")
        {
            _sPs.push_back(SKIN_LEFT_HAND);
            _sPs.push_back(SKIN_RIGHT_HAND);
        }
        else if (type=="all")
        {
            _sPs.push_back(SKIN_LEFT_FOREARM);
            _sPs.push_back(SKIN_RIGHT_FOREARM);
            _sPs.push_back(SKIN_LEFT_HAND);
            _sPs.push_back(SKIN_RIGHT_HAND);
        }
        else
        {
            yError("[doubleTouch] ERROR: type option was not among the admissible values!");
            return false;
        }

        verbosity = rf.check("verbosity", Value(0)).asInt();
        yInfo("[doubleTouch] verbosity set to %i", verbosity);

        rate = rf.check("rate", Value(100)).asInt();
        yInfo("[doubleTouch] rateThread working at %i ms.",rate);

        record = rf.check("record", Value(0)).asInt();
        yInfo("[doubleTouch] record variable is set to %i",record);

        filename = rf.check("filename", Value(".txt")).asString();
        yInfo("[doubleTouch] Module filename set to %s", filename.c_str());

        jnt_vels = rf.check("jnt_vels", Value(10.0)).asDouble();
        yInfo("[doubleTouch] Module jnt_vels set to %g", jnt_vels);

        color = rf.check("color", Value("white")).asString();
        yInfo("[doubleTouch] Robot color set to %s", color.c_str());

        Bottle &bHandConf=rf.findGroup("hand_configuration");


        // PAY ATTENTION HERE //
        if (!bHandConf.isNull())
        {
            bHandConf.setMonitor(rf.getMonitor());
            
            if (bHandConf.check("master"))
            {
                Bottle *bottleMaster=bHandConf.find("master").asList();
                handPossM = iCub::skinDynLib::vectorFromBottle(*bottleMaster,0,9);
                yInfo("[doubleTouch] Initializing master hand configuration: %s",
                        handPossM.toString(3,3).c_str());
            }
            else yInfo("[doubleTouch] Could not find [master] option in the config file; set to default.");

            if (bHandConf.check("slave"))
            {
                Bottle *bottleSlave=bHandConf.find("slave").asList();
                handPossS = iCub::skinDynLib::vectorFromBottle(*bottleSlave,0,9);
                yInfo("[doubleTouch] Initializing slave hand configuration: %s",
                        handPossS.toString(3,3).c_str());
            }
            else yInfo("[doubleTouch] Could not find [slave] option in the config file; set to default.");
        }
        else
        {
            yInfo("[doubleTouch] Could not find [hand_configuration] group in the config file; set all to default."); 
        }   
            
        time_t now = time(0);
        tm *ltm = localtime(&now);
        string time = int_to_string(1900 + ltm->tm_year)+"_"+int_to_string(1+ltm->tm_mon)+"_"+
                      int_to_string(ltm->tm_mday)+"_"+int_to_string(1+ltm->tm_hour)+"_"+
                      int_to_string(1+ltm->tm_min)+"_";

        // check if file are important or not
        if (record==2)
        {
            filename = "../calibration_data/"+time+filename;
        }
        else if (record==1)
        {
            filename = "../vRFlearning_data/"+time+filename;
        }
        else
        {
            filename = "../data/"+time+filename;
        }
        yInfo("[doubleTouch] Storing file set to: %s",filename.c_str());

        if (alignEyes)
        {
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);
        }
        else
        {
            doubleTouch_Thrd = new doubleTouchThread(rate, name, robot, verbosity,
                        _sPs, jnt_vels, record, filename, color, autoconnect, dontgoback, handPossM, handPossS);
            bool strt = doubleTouch_Thrd -> start();
            if (!strt)
            {
                delete doubleTouch_Thrd;
                doubleTouch_Thrd = 0;
                yError("ERROR!!! doubleTouchThread wasn't instantiated!!");
                return false;
            }
        }

        return true;
    }

    /*********************************************************************/
    bool close()
    {
        yInfo("DOUBLE TOUCH: Stopping threads..");
        if (doubleTouch_Thrd)
        {
            doubleTouch_Thrd->stop();
            delete doubleTouch_Thrd;
            doubleTouch_Thrd=0;
        }

        return true;
    }

    double getPeriod()  { return 1.0; }
    bool updateModule() { return true; }
};

/*********************************************************************/
int main(int argc, char * argv[])
{
    Network yarp;

    
    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("periPersonalSpace");
    rf.setDefaultConfigFile("doubleTouch.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(" "); 
        yInfo("Options:");
        yInfo(" ");
        yInfo("   --context     path:  where to find the called resource");
        yInfo("   --from        from:  the name of the .ini file.");
        yInfo("   --name        name:  the name of the module (default doubleTouch).");
        yInfo("   --robot       robot: the name of the robot. Default icubSim.");
        yInfo("   --rate        rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity   int:   verbosity level (default 0).");
        yInfo("   --record      int:   if to record data or not.");
        yInfo("      --record 0 -> nothing is recorded, the double touch is iterating over and");
        yInfo("                    over again. Demonstrative and testing purposes.");
        yInfo("      --record 1 -> recording for visuo-tactile reference frames purposes.");
        yInfo("      --record 2 -> recording for kinematic calibration purposes.");
        yInfo("   --dontgoback  flag: nothing is recorded. The double touch is executed once.");
        yInfo("                       The robot does not come back to a resting position.");
        yInfo("   --color       color: robot color (black or white - MANDATORY!)");
        yInfo("   --type        type:  the type of task (default 'LtoR').");
        yInfo("                        Allowed type names: 'RtoL','LtoR','RHtoL','LHtoR'");
        yInfo("                        Combinations: 'all','all_LtoR','all_RtoL','all_12DoF','all_14DoF'");
        yInfo("   --filename    file:  the name of the file to be saved in case of");
        yInfo("                        a recording session. Default 'calibration.txt'.");
        yInfo("                        A date is appended at the beginning for completeness.");
        yInfo("   --autoconnect flag: if or not to autoconnect to the skinManager");
        yInfo("   --jnt_vels    double: specify the joint level speed during the double touch. Default 4[deg/s].");
        yInfo("   --alignEyes   flag: if or not to use the rpc-thing and sync with alignEyes module.");
        yInfo(" ");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    doubleTouch dblTch;
    return dblTch.runModule(rf);
}

