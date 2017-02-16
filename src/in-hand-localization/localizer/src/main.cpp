#include <iostream>
#include <fstream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include "../headers/localizer.h"
#include "../headers/scalingSeries.h"
#include "../headers/unscentedParticleFilter.h"

#include "../build/include/src/localizer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class localizingModule : public RFModule,
                         public localizer_IDL
{
    string module_name;
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
    bool acquire;
    bool pose_saved;
    bool pose_computed;
    bool enabled_touch;

    int down;
    int j_obj;
    int fileCount;
    int num_objs;
    int num_m_values;
    int num_particles;
    int num_Q, num_trials;

    ResourceFinder rf;

    Vector result;
    Matrix solutions;
    Vector error_indices;
    deque<Vector> measurements;

    Mutex mutex;

public:
    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /*******************************************************************************/
    bool go_localize()
    {
        localize=true;
        return true;
    }

    /*******************************************************************************/
    bool stop_localize()
    {
        localize=false;
        return true;
    }

    /*******************************************************************************/
    bool go_acquire()
    {
        acquire=true;
        Time::delay(0.5);

        if (measurements.size()>0)
            return true;
        else
            return false;
    }

    /*******************************************************************************/
    Bottle get_estimated_pose()
    {
        LockGuard lg(mutex);

        Bottle pose;
        if (pose_computed)
        {
            pose.addDouble(result[0]);pose.addDouble(result[1]);pose.addDouble(result[2]);
            pose.addDouble(result[3]);pose.addDouble(result[4]);pose.addDouble(result[5]);
            pose.addInt(j_obj);
        

            ofstream fout;
            stringstream fileName;
            fileName<<homeContextPath + "/" + savename<<"_"<<fileCount;
            string fileNameFormat;
            fileNameFormat=fileName.str()+".txt";
            fout.open(fileNameFormat.c_str());
            if (fout.is_open() && pose_saved==false)
            {
                fout<<"position"<<endl<<endl;
                fout<<result.subVector(0,2).toString()<<endl;
                fout<<"orientation"<<endl<<endl;
                fout<<result.subVector(3,5).toString()<<endl;
                pose_saved=true;

                cout<<endl<<" Pose saved in "<<fileNameFormat<<endl<<endl;
            }
            else
            {
                pose_saved=false;
            }
        }

        return pose;
    }

    /*******************************************************************************/
    Bottle get_solution_statistics(const int i)
    {
        Bottle sol;
        Bottle &bsol=sol.addList();
        if (pose_computed)
        {
            cout<<endl;

            for (size_t j=0; j<solutions.getCol(0).size(); j++)
            {
                 cout<<" Solution "<<j<<endl;
                 cout<<" Localization error: "<< solutions(j,0)<<endl;
                 cout<<" Execution time    : "<< solutions(j,1)<<endl;
            }
            cout<<endl;

           bsol.addString("Localization error"); bsol.addDouble(solutions(i,0));
           bsol.addString("Execution time"); bsol.addDouble(solutions(i,1));
        }

        return sol;
    }


    /*******************************************************************************/
    Bottle get_meas()
    {
        Bottle bmeas;

        if (measurements.size()>0)
        {
            for (size_t i=0; i<measurements.size(); i++)
            {
                Bottle &bbmeas=bmeas.addList();
                bbmeas.addDouble(measurements[i][0]); bbmeas.addDouble(measurements[i][1]); bbmeas.addDouble(measurements[i][2]);
            }
        }
        else
        {
            yError()<< "No measurements received!";
            bmeas.addList();
        }

        return bmeas;
    }

    /*******************************************************************************/
    bool set_object_name(const string &object)
    {
        object_name=object;
        if (object=="domino")
            j_obj=0;
        else if (object=="jello")
            j_obj=1;
        else if (object=="soup")
            j_obj=2;
        else if (object=="meat")
            j_obj=3;
        else
            return false;

        cout<<" Object selected "<< j_obj<<endl;

        return true;
    }

    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("in-hand-localizer"), "Getting module name").asString();
        object_name=rf.check("module_name", Value("object"), "Getting module name").asString();
        online=(rf.check("online", Value("no")).asString()=="yes");
        enabled_touch=(rf.check("enabled_touch", Value("no")).asString()=="yes");

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

        pose_computed=false;
        acquire=false;
        pose_saved=false;

        j_obj=0;

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
        if (online && acquire)
        {
            askForPoints();
        }

        //for (size_t j=0; j<num_objs;j++)
        //{
            for (size_t k=0; k<num_m_values; k++)
            {
                for (size_t l=0; l<num_particles; l++)
                {
                    for (size_t m=0; m<num_Q; m++)
                    {
                        solutions.resize(num_trials,4);
                        for(size_t i=0; i<num_trials; i++)
                        {
                            if (localize)
                            {
                                if (measurements.size()>0)
                                {
                                    Localizer *loc5=new UnscentedParticleFilter();
                                    //if (loc5->configure(this->rf,j,k, num_m_values, l, num_particles, m, online, measurements, enabled_touch))
                                    if (loc5->configure(this->rf,j_obj,k, num_m_values, l, num_particles, m, online, measurements, enabled_touch))
                                    {
                                        error_indices=loc5->localization();
                                        result=error_indices.subVector(0,5);
                                        loc5->saveData(error_indices,i,k,l,m);
                                        solutions(i,0)=error_indices[6];
                                        solutions(i,1)=error_indices[7];
                                        solutions(i,2)=error_indices[8];
                                        solutions(i,3)=error_indices[9];

                                        cout<<endl<<endl<<" Solution computed"<<endl;
                                        cout<<" Localization error: "<< error_indices[6]<<endl;
                                        cout<<" Execution time    : "<< error_indices[7]<<endl<<endl;

                                        delete loc5;
                                        pose_computed=true;
                                        pose_saved=false;
                                    }
                                    else
                                    {
                                        localize=false;
                                        delete loc5;
                                        return false;
                                    }
                                }
                                else
                                    yError()<< " No measurements received!! ";
                            }
                        }

                        Localizer *loc5=new UnscentedParticleFilter();
                        //loc5->configure(this->rf,j, k, num_m_values, l, num_particles,m, online, measurements, enabled_touch);
                        if (loc5->configure(this->rf,j_obj,k, num_m_values, l, num_particles, m, online, measurements, enabled_touch))
                        //loc5->saveStatisticsData(solutions,j,k,l,m);
                       loc5->saveStatisticsData(solutions,j_obj,k,l,m);


                        delete loc5;
                    }

                }
            }
        //}

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
        Bottle cmd, reply;
        cmd.addString("get_filtered_points");

        Vector point(3,0.0);
        measurements.clear();

        if (portPointsRpc.write(cmd,reply))
        {
            Bottle *bpoint=reply.get(0).asList();
            for (size_t i=0; i<bpoint->size(); i++)
            {
                Bottle *blist=bpoint->get(i).asList();
                point[0]=blist->get(0).asDouble();
                point[1]=blist->get(1).asDouble();
                point[2]=blist->get(2).asDouble();
                measurements.push_back(point);
                //cout<<point.toString()<<endl;

            }

            cout<<endl<<" " <<measurements.size()<< " points have been received"<<endl<<endl;
            acquire=false;
        }

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
    rf.setDefaultContext("localizerHandover");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
