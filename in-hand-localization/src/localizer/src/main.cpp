#include <iostream>
#include <fstream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include "../headers/localizer.h"
#include "../headers/scalingSeries.h"
#include "../headers/unscentedParticleFilter.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class localizingModule : public RFModule
{
    string module_name;
    string save_name;
    string homeContextPath;
    string savename;
    string fileOutFormat;
    string object_name;
    string fileInName;
    string algorithm;

    vector<Vector> points;

    RpcServer portRpc;

    RpcClient portPointsRpc;
    RpcServer portPoseRpc;

    bool online;
    bool saving;
    bool localize;
    bool enabled_touch;

    int down;
    int fileCount;
    int num_objs;
    int num_m_values;
    int num_particles;
    int num_Q, num_trials;

    ResourceFinder rf;

    Matrix solutions;
    Vector error_indices;
    deque<Vector> measurements;

public:
    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("inHandLocalizer"), "Getting module name").asString();
        object_name=rf.check("module_name", Value("object"), "Getting module name").asString();
        online=(rf.check("online", Value("no")).asString()=="yes");
        enabled_touch=(rf.check("touch", Value("no")).asString()=="yes");

        homeContextPath=rf.getHomeContextPath().c_str();
        savename=rf.check("savename", Value(object_name+"_pose"), "Default file savename").asString();
        fileOutFormat=rf.check("format", Value("off"), "Default file format").asString();
        saving=(rf.check("savingClouds", Value("yes"), "Toggle save clouds as file").asString()=="yes");
        down= rf.check("downsampling", Value(1)).asInt();
        algorithm=rf.check("algorithm", Value("mupf"), "Default file format").asString();
        num_objs=rf.check("num_objs", Value(1)).asInt();
        num_m_values=rf.check("num_m_values", Value(1)).asInt();
        num_particles=rf.check("num_particles", Value(1)).asInt();
        num_objs=rf.check("num_objs", Value(1)).asInt();
        num_Q=rf.check("num_Q", Value(1)).asInt();
        num_trials=rf.check("num_trials", Value(1)).asInt();

        if (online)
            localize=false;
        else
            localize=true;

        cout<<"Poses will be saved in "<<homeContextPath<<" folder, as "<<savename<<"N."<<fileOutFormat<<", with increasing numeration N"<< endl;
        fileCount=0;

        if (online)
        {
            portPointsRpc.open("/" + module_name + "/pnt:i");
            portPoseRpc.open("/" + module_name + "/ps:o");
        }

        portRpc.open("/" + module_name + "/rpc");
        attach(portRpc);

        this->rf=rf;

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        if (online)
        {
            portPointsRpc.interrupt();
            portPoseRpc.interrupt();
        }

        portRpc.close();

        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        if (online)
        {
            portPointsRpc.close();
            portPoseRpc.close();
        }

        portRpc.interrupt();

        return true;
    }

    /*******************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /*******************************************************************************/
    bool updateModule()
    {
        if (online)
        {
            askForPoints();
        }

        if (localize)
        {
            for (size_t j=0; j<num_objs;j++)
            {
                for (size_t k=0; k<num_m_values; k++)
                {
                    for (size_t l=0; l<num_particles; l++)
                    {
                        for (size_t m=0; m<num_Q; m++)
                        {
                            solutions.resize(num_trials,4);
                            for(size_t i=0; i<num_trials; i++)
                            {
                                Localizer *loc5=new UnscentedParticleFilter();
                                loc5->configure(this->rf,j,k, num_m_values, l, num_particles, m, online, measurements, enabled_touch);
                                error_indices=loc5->localization();
                                loc5->saveData(error_indices,i,k,l,m);
                                solutions(i,0)=error_indices[6];
                                solutions(i,1)=error_indices[7];
                                solutions(i,2)=error_indices[8];
                                solutions(i,3)=error_indices[9];

                                delete loc5;
                            }

                            Localizer *loc5=new UnscentedParticleFilter();
                            loc5->configure(this->rf,j, k, num_m_values, l, num_particles,m, online, measurements, enabled_touch);
                            loc5->saveStatisticsData(solutions,j,k,l,m);

                            delete loc5;
                        }
                    }
                }
            }           
        }

        if (online)
        {
            localize=false;
            return true;
        }
        else
            return false;
    }

    /*******************************************************************************/
    bool askForPoints()
    {
        return true;
    }
};

/*******************************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    localizingModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("cloudFiltering");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
