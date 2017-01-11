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

    int verbosity,rate,record;
    bool dontgoback;
    bool go;
    bool automatic_start;

    double jnt_vels;
    
    Vector handPossM;
    Vector handPossS;

public:
    doubleTouch()
    {
        doubleTouch_Thrd=0;

        robot    = "icubSim";
        name     = "closed-chain";
        type     = "LHtoR";
        filename = ".txt";

        verbosity =    0;
        rate      =  100;
        record    =    0;
        jnt_vels  = 10.0;

        dontgoback  = false;
        
        handPossM.resize(9,0.0);

        handPossM[0]=80.0; handPossM[1]=0.0;
        handPossM[2]=0.0; handPossM[3]=0.0;
        handPossM[4]=0.0;  handPossM[5]=0.0;
        handPossM[6]=0.0; handPossM[7]=0.0;

        handPossS.resize(9,0.0);
        handPossS[0]=40.0;  handPossS[1]=0.0;
        handPossS[2]=0.0;   handPossS[3]=0.0;
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
                case VOCAB4('s','t','a','r'):
                {
                    doubleTouch_Thrd = new doubleTouchThread(rate, name, robot, verbosity,
                                                       jnt_vels, record, filename,
                                                        dontgoback, handPossM, handPossS, go, automatic_start);
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

                case VOCAB4('s','t','o','p'):
                {
                    if (doubleTouch_Thrd)
                    {
                        yInfo(" Stopping threads..");
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
        dontgoback     = rf.check("dontgoback");

        cout<<endl;

        if (dontgoback)
        {
            yInfo(" Dontgoback flag set to ON");
        }

        name = rf.check("name", Value("double-touch")).asString();
        yInfo(" Module name set to %s", name.c_str());

        setName(name.c_str());

        robot = rf.check("robot", Value("icubSim")).asString();
        yInfo(" Robot is: %s", robot.c_str());

        type = rf.check("type", Value("RHtoL")).asString();
        yInfo(" Type is: %s", type.c_str());

        verbosity = rf.check("verbosity", Value(0)).asInt();
        yInfo(" Verbosity set to %i", verbosity);

        rate = rf.check("rate", Value(100)).asInt();
        yInfo(" RateThread working at %i ms.",rate);

        record = rf.check("record", Value(0)).asInt();
        yInfo(" Record variable is set to %i",record);

        filename = rf.check("filename", Value(".txt")).asString();
        yInfo(" Module filename set to %s", filename.c_str());

        jnt_vels = rf.check("jnt_vels", Value(10.0)).asDouble();
        yInfo(" Module jnt_vels set to %g", jnt_vels);

        go=(rf.check("closed_chain", Value("no")).asString()== "no");

        automatic_start=(rf.check("automatic_start", Value("no")).asString()== "no");

        time_t now = time(0);
        tm *ltm = localtime(&now);
        string time = int_to_string(1900 + ltm->tm_year)+"_"+int_to_string(1+ltm->tm_mon)+"_"+
                      int_to_string(ltm->tm_mday)+"_"+int_to_string(1+ltm->tm_hour)+"_"+
                      int_to_string(1+ltm->tm_min)+"_";

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
        yInfo(" Storing file set to: %s",filename.c_str());

        if (alignEyes)
        {
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);
        }
        else
        {
            doubleTouch_Thrd = new doubleTouchThread(rate, name, robot, verbosity,
                               jnt_vels, record, filename, dontgoback, handPossM, handPossS,go, automatic_start);
            bool strt = doubleTouch_Thrd -> start();
            if (!strt)
            {
                delete doubleTouch_Thrd;
                doubleTouch_Thrd = 0;
                yError("ERROR!!! doubleTouchThread wasn't instantiated!!");
                return false;
            }
        }
        cout<<endl;

        return true;
    }

    /*********************************************************************/
    bool close()
    {
        yInfo(" Stopping threads..");
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
        yInfo("   --filename    file:  the name of the file to be saved in case of");
        yInfo("                        a recording session. Default 'calibration.txt'.");
        yInfo("                        A date is appended at the beginning for completeness.");
        yInfo("   --jnt_vels    double: specify the joint level speed during the double touch. Default 4[deg/s].");
        yInfo("   --alignEyes   flag: if or not to use the rpc-thing and sync with alignEyes module.");
        yInfo(" ");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf(" No Network!!!\n");
        return -1;
    }

    doubleTouch dblTch;
    return dblTch.runModule(rf);
}

