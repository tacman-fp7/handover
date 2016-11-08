#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "pointCloudFiltering_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class filteringModule : public RFModule,
                        public pointCloudFiltering_IDL
{
    bool online;
    string module_name;
    string homeContextPath;
    string savename;
    string fileOutFormat;
    string fileInFormat;
    string fileInName;
    string tactPoseFileName;
    string fingersFileName;
    string info;

    bool saving;
    int fileCount;
    int down;

    RpcServer portRpc;

    RpcClient portCloudRpc;
    RpcClient portTactRpc;

    vector<Vector> pointsIn;
    vector<Vector> pointsOut;
    vector<Vector> fingers;
    Matrix H_hand;

    double radius;
    int nnThreshold;
    int numVertices;
    double radius_color;
    int nnThreshold_color;
    int diff_rgb;
    int diff_ycbcr;

    bool spatial_filter;
    bool gray_filter;
    bool change_frame;
    bool hand_filter;
    bool volume_filter;
    bool ellips_filter;
    bool go_on;
    string color_space;
    double x_lim, y_lim, z_lim;

    Vector center_volume;
    double radius_volume;
    Vector center_ellips;
    double a,b;
    double radius_volume_offset;
    double a_offset, b_offset;
    Vector position, orientation;

public:
    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /***********************************************************************/
    bool set_saving(const string &entry)
    {
        if (entry=="yes" || entry=="no")
        {
            saving=(entry=="yes");
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool set_filename(const string &entry)
    {
        savename=entry;
        return true;
    }

    /***********************************************************************/
    Bottle get_filtered_points()
    {
        Bottle bpoints;

        vector<Vector> pointsTobeSent;

        if ((spatial_filter==true || gray_filter==true || volume_filter==true)==true && pointsOut.size()>0)
            pointsTobeSent=pointsOut;
        else
            pointsTobeSent=pointsIn;

        if (pointsTobeSent.size()>0)
        {
            for (size_t i=0; i<pointsTobeSent.size(); i++)
            {
                Vector point=pointsTobeSent[i];
                Bottle &bpoint=bpoints.addList();
                bpoint.addDouble(point[0]); bpoint.addDouble(point[1]);bpoint.addDouble(point[2]);
            }
        }
        else
            yError()<<"No points available!";

        return bpoints;
    }

    /***********************************************************************/
    string get_filters()
    {
        string filters;

        if (hand_filter)
            filters="HF";
        if (gray_filter)
            filters+=" GF";
        if (spatial_filter)
            filters+=" SF";
        if (volume_filter)
            filters+=" VF";

        return filters;
    }

    /***********************************************************************/
    bool set_hand_filter(const string &entry)
    {
        if (entry=="yes" || entry=="no")
        {
            hand_filter=(entry=="yes");
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool set_gray_filter(const string &entry)
    {
        if (entry=="yes" || entry=="no")
        {
            gray_filter=(entry=="yes");
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool set_spatial_filter(const string &entry)
    {
        if (entry=="yes" || entry=="no")
        {
            spatial_filter=(entry=="yes");
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool set_volume_filter(const string &entry)
    {
        if (entry=="yes" || entry=="no")
        {
            volume_filter=(entry=="yes");
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool set_all_filters(const string &entry)
    {
        if (entry=="yes")
        {
            hand_filter=true;
            gray_filter=true;
            spatial_filter=true;
            volume_filter=true;
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool go()
    {
        go_on=true;
        return true;
    }   


    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("cloud-filtering"), "Getting module name").asString();

        online=(rf.check("online", Value("no")).asString()=="yes");

        homeContextPath=rf.getHomeContextPath().c_str();
        savename=rf.check("savename", Value("filtered-cloud"), "Default file savename").asString();
        saving=(rf.check("savingClouds", Value("yes"), "Toggle save clouds as file").asString()=="yes");
        fileOutFormat=rf.check("format", Value("off"), "Default file format").asString();
        down= rf.check("downsampling", Value(1)).asInt();
        fileInFormat="off"; // to be extended
        if (online)
            go_on=false;

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<savename<<"N."<<fileOutFormat<<", with increasing numeration N"<< endl;
        fileCount=0;

        spatial_filter=(rf.check("spatial_filter", Value("no")).asString()=="yes");
        gray_filter=(rf.check("gray_filter", Value("yes")).asString()=="yes");
        color_space=rf.check("color_code", Value("rgb")).asString();
        radius=rf.check("radius", Value(0.0002)).asDouble();
        nnThreshold=rf.check("nn-threshold", Value(40)).asInt();
        radius_color=rf.check("radius_color", Value(0.0003)).asDouble();
        nnThreshold_color=rf.check("nn-threshold_color", Value(10)).asInt();
        diff_rgb=rf.check("diff_rgb", Value(25)).asInt();
        diff_ycbcr=rf.check("diff_ycbcr", Value(2)).asInt();

        change_frame=(rf.check("change_frame", Value("yes")).asString()=="yes");
        hand_filter=(rf.check("hand_filter", Value("no")).asString()=="yes");
        volume_filter=(rf.check("volume_filter", Value("no")).asString()=="yes");
        ellips_filter=(rf.check("ellips_filter", Value("no")).asString()=="yes");
        x_lim=rf.check("x_lim", Value(0.18)).asDouble();
        y_lim=rf.check("y_lim", Value(0.18)).asDouble();
        z_lim=rf.check("z_lim", Value(0.18)).asDouble();
 
        radius_volume_offset=rf.check("radius_volume_offset", Value(0.03)).asDouble();
        a_offset=rf.check("a_offset", Value(0.03)).asDouble();
        cout<<"a offset "<<a_offset<<endl;
        b_offset=rf.check("b_offset", Value(0.0)).asDouble();

        center_volume.resize(2,0.0);
        center_ellips.resize(2,0.0);
        position.resize(3,0.0);
        orientation.resize(4,0.0);

        if (online)
        {
            portCloudRpc.open("/" + module_name + "/cld:i");
            portTactRpc.open("/" + module_name + "/tct:i");
        }
        else
        {
            fileInName=rf.check("fileInName", Value("cloud.off"), "Default cloud name").asString();
            if (change_frame)
            {
                tactPoseFileName=rf.check("tactPoseFileName", Value("hand_pose.off"), "Default cloud name").asString();
                H_hand=readPose();
            }

            if (volume_filter)
            {
                fingersFileName=rf.check("fingersFileName", Value("tactile_hand_pose.off"), "Default cloud name").asString();
                readPointCloud(fingersFileName, "fingers");
                graspableVolume(fingers);
            }
            if (ellips_filter)
            {
                fingersFileName=rf.check("fingersFileName", Value("tactile_hand_pose.off"), "Default cloud name").asString();
                readPointCloud(fingersFileName, "fingers");
                graspableEllips(fingers);
            }

            return readPointCloud(fileInName, "points");
        }

        portRpc.open("/"+module_name+"/rpc");
        attach(portRpc);

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        if (online)
        {
            portCloudRpc.interrupt();
            portTactRpc.interrupt();
        }

        portRpc.close();

        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        if (online)
        {
            portCloudRpc.close();
            portTactRpc.close();
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
        Vector colors(3,0.0);

        if (online && pointsIn.size()==0 && go_on)
        {
            askForCloud();
        }

        if (online==false)
            go_on=true;

        if (pointsIn.size()>0 && go_on)
        {
            askForFingers();
            askForPose();

            if (change_frame)
                fromRobotTohandFrame(pointsIn);

            if (hand_filter)
            {
                colors[1]=255;
                handFilter(pointsIn, x_lim, y_lim, z_lim);
                info="_HF";
                saveNewCloud(colors, pointsIn, info);
            }

            if (gray_filter && color_space == "rgb" && pointsIn.size()>0)
            {
                colors[2]=255;
                colors[1]=0;
                grayColorFilter(radius_color,nnThreshold_color,diff_rgb, pointsIn);
                info+="_GF_rgb";
                saveNewCloud(colors, pointsOut, info);
            }
            else if (gray_filter && color_space == "ycbcr" && pointsIn.size()>0)
            {
                colors[2]=255;
                colors[1]=0;
                grayColorFilter(radius_color,nnThreshold_color,diff_ycbcr, pointsIn);
                info+="_GF_ycbcr";
                saveNewCloud(colors, pointsOut, info);
            }

            if (spatial_filter && pointsIn.size()>0)
            {
                colors[0]=255;
                colors[2]=0;
                if (gray_filter)
                    spatialDensityFilter(radius,nnThreshold+1, pointsOut);
                else
                    spatialDensityFilter(radius,nnThreshold+1, pointsIn);
                info+="_SF";
                saveNewCloud(colors, pointsOut, info);
            }

            if (volume_filter && pointsIn.size()>0 )
            {
                colors[1]=255;
                if ((spatial_filter==true || gray_filter==true)==true)
                    volumeFilter(pointsOut);
                else
                    volumeFilter(pointsIn);
                info+="_VF";

                saveNewCloud(colors, pointsOut,info);
            }

            if (ellips_filter && pointsIn.size()>0 )
            {
                colors[1]=255;
                if ((spatial_filter==true || gray_filter==true)==true)
                    ellipsFilter(pointsOut);
                else
                    ellipsFilter(pointsIn);
                info+="_EF";

                saveNewCloud(colors, pointsOut,info);
            }

            if (info == "_HF_GF_rgb_SF_VF" ||info == "_HF_GF_ycbcr_SF_VF"||info == "_HF_GF_ycbcr_SF_EF"|| info == "_HF_GF_ycbcr_SF_EF")
            {
                info="all_filters";
                saveNewCloud(colors, pointsOut,info);
            }            

            pointsIn.clear();
            go_on=false;
            fileCount++;
        }
        else
        {
            if (online)
                return true;
            else
                return false;
        }
    }

    /*******************************************************************************/
    bool readPointCloud(string &filename, string what)
    {
        int state=0;
        int nPoints;
        char line[255];
        deque<Vector> points_tmp;
        Vector point_tmp;
        point_tmp.resize(6,0.0);

        cout<< "In cloud file "<<homeContextPath+"/"+filename<<endl;

        ifstream cloudFile((homeContextPath+"/"+filename).c_str());
        if (!cloudFile.is_open())
        {
            yError()<<"problem opening cloud file!";
            return false;
        }

        while (!cloudFile.eof())
        {
            cloudFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
                if (tmp=="COFF" || tmp=="OFF")
                    state++;
            }
            else if (state==1)
            {
                if (isNumber)
                {
                    nPoints=firstItem.asInt();
                    state++;
                }
            }
            else if (state==2)
            {
                if (isNumber && (b.size()>=3))
                {
                    point_tmp[0]=b.get(0).asDouble();
                    point_tmp[1]=b.get(1).asDouble();
                    point_tmp[2]=b.get(2).asDouble();
                    point_tmp[3]=b.get(3).asDouble();
                    point_tmp[4]=b.get(4).asDouble();
                    point_tmp[5]=b.get(5).asDouble();
                    points_tmp.push_back(point_tmp);

                    if (--nPoints<=0)
                    {
                        for( size_t i=0; i<points_tmp.size();i=i+down)
                        {
                            point_tmp=points_tmp[i];
                            //cout<<"point "<<point_tmp.toString(3,3).c_str()<<endl;
                            if (what=="fingers")
                            {
                                fingers.push_back(point_tmp);
                            }
                            else
                                pointsIn.push_back(point_tmp);
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /*******************************************************************************/
    bool askForCloud()
    {
        Bottle cmd,reply;
        cmd.addString("get_3D_blob_points");
        pointsIn.clear();

        if(portCloudRpc.write(cmd,reply))
        {
            Bottle *bbottle=reply.get(0).asList();
            for (int i=0; i<bbottle->size(); i+=down)
            {
                Bottle *bcloud=bbottle->get(i).asList();
                Vector aux(6,0.0);
                aux[0]=bcloud->get(0).asDouble();
                aux[1]=bcloud->get(1).asDouble();
                aux[2]=bcloud->get(2).asDouble();
                aux[3]=bcloud->get(3).asInt();
                aux[4]=bcloud->get(4).asInt();
                aux[5]=bcloud->get(5).asInt();

                pointsIn.push_back(aux);

            }

            if (pointsIn.size()<0)
            {
                yError()<<"No blob points retrieved!";
                return false;
            }
        }
        else
        {
            yError()<<"cloud Extractor didn't reply!";
            return false;
        }

        return true;
    }

    /*******************************************************************************/
    bool askForFingers()
    {
        Bottle cmd,reply;
        cmd.addString("get_tactile_data");

        if (portTactRpc.write(cmd,reply))
        {
            Bottle *bbottle=reply.get(0).asList();
            for (int i=0; i<bbottle->size(); i++)
            {
                Bottle *bcloud=bbottle->get(i).asList();
                Vector aux(3,0.0);
                if (bcloud->get(0).asString()=="thumb" || bcloud->get(0).asString()=="index" ||bcloud->get(0).asString()=="middle")
                {
                    aux[0]=bcloud->get(1).asDouble();
                    aux[1]=bcloud->get(2).asDouble();
                    aux[2]=bcloud->get(3).asDouble();
                    fingers.push_back(aux);
                }
            }

            if (fingers.size()<0)
            {
                yError()<<"No finger positions retrieved!";
                return false;
            }
        }
        else
        {
            yError()<<"kinematics didn't reply!";
            return false;
        }

        return true;
    }

    /*******************************************************************************/
    Matrix askForPose()
    {
        Matrix H;
        Bottle cmd,reply;
        cmd.addString("get_pose");

        if (portTactRpc.write(cmd,reply))
        {
            Bottle *bbottle=reply.get(0).asList();
            for (int i=0; i<bbottle->size(); i++)
            {
                Bottle *bcloud=bbottle->get(i).asList();
                if (bcloud->get(0).asString()=="position")
                {
                    position[0]=(bcloud->get(1).asDouble());
                    position[1]=(bcloud->get(2).asDouble());
                    position[2]=(bcloud->get(3).asDouble());
                }
                if (bcloud->get(0).asString()=="orientation")
                {
                    orientation[0]=(bcloud->get(1).asDouble());
                    orientation[1]=(bcloud->get(2).asDouble());
                    orientation[2]=(bcloud->get(3).asDouble());
                    orientation[3]=(bcloud->get(4).asDouble());
                }
            }

            H.resize(4,0.0);

            H=axis2dcm(orientation);
            Vector x_aux(4,1.0);
            x_aux.setSubvector(0,position);
            H.setCol(3,x_aux);
            H=SE3inv(H);
        }
        else
        {
            yError()<<"kinematics didn't reply!";
        }

        return H;
    }


    /*******************************************************************************/
    vector<int>  spatialDensityFilter(const double radius, const int maxResults, vector<Vector> &points)
    {
        numVertices=points.size();

        vector<Vector> pointsTmp=points;
        pointsOut.clear();

        cv:: Mat data(numVertices,3,CV_32F);
        for (int i=0; i<numVertices; i++)
        {
            Vector point=pointsTmp[i];
            data.at<float>(i,0)=(float)point[0];
            data.at<float>(i,1)=(float)point[1];
            data.at<float>(i,2)=(float)point[2];
        }

        cv::flann::KDTreeIndexParams indexParams;
        cv::flann::Index kdtree(data,indexParams);

        cv::Mat query(1,data.cols,CV_32F);
        cv::Mat indices,dists;

        vector<int> res(data.rows);

        for (size_t i=0; i<res.size(); i++)
        {
            for (int c=0; c<query.cols; c++)
                query.at<float>(0,c)=data.at<float>(i,c);

            res[i]=kdtree.radiusSearch(query,indices,dists,radius,maxResults,cv::flann::SearchParams(250));

            Vector point(3,0.0);
            if (res[i]>=maxResults)
            {
                point[0]=data.at<float>(i,0);
                point[1]=data.at<float>(i,1);
                point[2]=data.at<float>(i,2);
                pointsOut.push_back(point);
            }
        }

        return res;
    }

    /*******************************************************************************/
    vector<int>  grayColorFilter(const double radius, const int maxResults, const int diff_colors, vector<Vector> &pointsIn)
    {
        numVertices=pointsIn.size();
        pointsOut.clear();

        cv:: Mat data(numVertices,3,CV_32F);
        cv:: Mat datacolor(numVertices,3,CV_32F);

        if (color_space == "ycbcr")
            fromRGBtoYCbCr();

        for (int i=0; i<numVertices; i++)
        {
            Vector point=pointsIn[i];
            data.at<float>(i,0)=(float)point[0];
            data.at<float>(i,1)=(float)point[1];
            data.at<float>(i,2)=(float)point[2];

            datacolor.at<float>(i,0)=(float)point[3];
            datacolor.at<float>(i,1)=(float)point[4];
            datacolor.at<float>(i,2)=(float)point[5];
        }

        cv::flann::KDTreeIndexParams indexParams;
        cv::flann::Index kdtree(data,indexParams);

        cv::Mat query(1,data.cols,CV_32F);
        cv::Mat indices,dists;

        vector<int> res(data.rows);
        double average_diff, diff1, diff2, diff3;
        int count =0;
        average_diff=0;

        for (size_t i=0; i<res.size(); i++)
        {
            for (int c=0; c<query.cols; c++)
                query.at<float>(0,c)=data.at<float>(i,c);

            res[i]=kdtree.radiusSearch(query,indices,dists,radius,maxResults,cv::flann::SearchParams(128));

            int count_index=0;
            for (int j=0; j<indices.cols; j++)
            {
                if (abs(indices.at<int>(0,j)) <=datacolor.rows)
                {
                    count_index++;
                    if (color_space == "rgb")
                    {
                        diff1 = abs(datacolor.at<float>(indices.at<int>(0,j),0) - datacolor.at<float>(indices.at<int>(0,j),1));
                        diff2 = abs(datacolor.at<float>(indices.at<int>(0,j),2) - datacolor.at<float>(indices.at<int>(0,j),1));
                        diff3 = abs(datacolor.at<float>(indices.at<int>(0,j),0) - datacolor.at<float>(indices.at<int>(0,j),2));
                        average_diff += (diff1+diff2+diff3)/3;
                    }
                    else
                    {
                        diff1 = abs(datacolor.at<float>(indices.at<int>(0,j),1) - 128);
                        diff2 = abs(datacolor.at<float>(indices.at<int>(0,j),2) - 128);
                        average_diff += (diff1+diff2)/2;
                    }
                }
            }
            average_diff/=count_index;

            Vector point(3,0.0);
            if (res[i]>=maxResults && average_diff>=diff_colors)
            {
                point[0]=data.at<float>(i,0);
                point[1]=data.at<float>(i,1);
                point[2]=data.at<float>(i,2);
                pointsOut.push_back(point);
                count++;
                average_diff=0.0;
            }
        }
        return res;
    }

    /*******************************************************************************/
    void handFilter(vector<Vector> &points, double &x_lim , double &y_lim, double &z_lim)
    {
        vector<Vector> pointsTmp=points;
        pointsIn.clear();

        for (size_t i=0; i<pointsTmp.size(); i++)
        {
            Vector point=pointsTmp[i];

            if ((abs(point[0])<= x_lim) && (abs(point[1])<= y_lim) && (abs(point[2])<= z_lim) )
                pointsIn.push_back(point);
        }
    }

    /*******************************************************************************/
    bool saveNewCloud(const Vector &colors, vector<Vector> &pointsToBeSaved, string &info)
    {
        ofstream fout;
        stringstream fileName;
        string fileNameFormat;
        fileName.str("");
        fileName << homeContextPath + "/" + savename.c_str() << fileCount<<info;

        if (fileOutFormat == "ply")
        {
            fileNameFormat = fileName.str()+".ply";
            cout << "Saving as " << fileNameFormat << endl;
            fout.open(fileNameFormat.c_str());
            if (fout.is_open())
            {
                fout << "ply\n";
                fout << "format ascii 1.0\n";
                fout << "element vertex " << pointsToBeSaved.size() <<"\n";
                fout << "property float x\n";
                fout << "property float y\n";
                fout << "property float z\n";
                fout << "property uchar diffuse_red\n";
                fout << "property uchar diffuse_green\n";
                fout << "property uchar diffuse_blue\n";
                fout << "end_header\n";

                for (unsigned int i=0; i<pointsToBeSaved.size(); i++)
                {
                    fout << pointsToBeSaved[i][0] << " " <<      pointsToBeSaved[i][1] << " " <<      pointsToBeSaved[i][2] << " " << (int)pointsToBeSaved[i][3] << " " << (int)pointsToBeSaved[i][4] << " " << (int)pointsToBeSaved[i][5] << "\n";
                    //plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";
                }

                fout.close();
                cout << "Points saved as " << fileNameFormat << endl;                
            }

        }
        else if (fileOutFormat == "off")
        {
            fileNameFormat = fileName.str()+".off";
            cout << "Saving as " << fileNameFormat << endl;
            fout.open(fileNameFormat.c_str());
            if (fout.is_open())
            {

                fout<<"COFF"<<endl;
                fout<<pointsToBeSaved.size()<<" 0 0"<<endl;
                fout<<endl;
                for (size_t i=0; i<pointsToBeSaved.size(); i++)
                {
                    if (info!="_HF")
                    {
                        fout<<pointsToBeSaved[i].subVector(0,2).toString(3,4).c_str()<<" "<<
                              colors[0]<<" "<<colors[1]<<" "<<colors[2]<<endl;
                    }
                    else
                        fout<<pointsToBeSaved[i].subVector(0,2).toString(3,4).c_str()<<" "<<pointsToBeSaved[i].subVector(3,5).toString(0,4).c_str()<<endl;
                }
                fout<<endl;
            }

            fout.close();
            cout << "Points saved as " << fileNameFormat << endl;
        }
        else if (fileOutFormat == "none")
        {
            cout << "Points not saved" << endl;
        }
    }

    /*******************************************************************************/
    void fromRGBtoYCbCr()
    {
        Vector *point;

        Matrix M_rgb2ycbcr(3,3);
        M_rgb2ycbcr(0,0)=0.299;
        M_rgb2ycbcr(0,1)=0.587;
        M_rgb2ycbcr(0,2)=0.144;
        M_rgb2ycbcr(1,0)=-0.168736;
        M_rgb2ycbcr(1,1)=-0.331264;
        M_rgb2ycbcr(1,2)=0.5;
        M_rgb2ycbcr(2,0)=0.5;
        M_rgb2ycbcr(2,1)=-0.418688;
        M_rgb2ycbcr(2,2)=-0.081312;

        Vector ycbcr(3,0.0);
        ycbcr[1]=128;
        ycbcr[2]=128;


        for (size_t i=0; i<pointsIn.size(); i++)
        {
            point=&pointsIn[i];
            cout<<"point "<<point->subVector(3,5).toString()<<endl;
            point->setSubvector(3,M_rgb2ycbcr*point->subVector(3,5) + ycbcr);

            cout<<"point computed "<<point->subVector(3,5).toString()<<endl;
        }
    }

    /*******************************************************************************/
    void fromRobotTohandFrame(vector<Vector> &pointstmp)
    {
        if (online)
        {
            H_hand=askForPose();
        }

        for (size_t i=0; i<pointstmp.size(); i++)
        {
            Vector aux(4,1.0);
            aux.setSubvector(0,pointstmp[i].subVector(0,2));
            aux=H_hand*(aux);
            pointstmp[i].setSubvector(0,aux.subVector(0,2));
        }
    }

    /*******************************************************************************/
    Matrix readPose()
    {
        Matrix H;
        int state=0;
        char line[255];

        cout<< "In pose file "<<homeContextPath+"/"+tactPoseFileName<<endl;

        ifstream poseFile((homeContextPath+"/"+tactPoseFileName).c_str());
        if (!poseFile.is_open())
        {
            yError()<<"problem opening pose file!";
        }

        while (!poseFile.eof())
        {
            poseFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
                if (tmp=="RIGHT" || tmp=="LEFT")
                    state++;
            }
            else if (state==1)
            {
                if (isNumber && (b.size()==3))
                {
                    position[0]=b.get(0).asDouble();
                    position[1]=b.get(1).asDouble();
                    position[2]=b.get(2).asDouble();
                }
                else if (isNumber && (b.size()==4))
                {
                    orientation[0]=b.get(0).asDouble();
                    orientation[1]=b.get(1).asDouble();
                    orientation[2]=b.get(2).asDouble();
                    orientation[3]=b.get(3).asDouble();
                }
            }
        }

        H.resize(4,0.0);

        H=axis2dcm(orientation);
        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,position);
        H.setCol(3,x_aux);
        H=SE3inv(H);

        return H;
    }

    /*******************************************************************************/
    void graspableVolume(vector<Vector> &fingerPoses)
    {
        Vector thumb, index, middle;
        thumb=fingerPoses[0];
        index=fingerPoses[1];
        middle=fingerPoses[2];
        double ma, mb;
        ma=(index[2]-thumb[2])/(index[0]-thumb[0]);
        mb=(middle[2]-index[2])/(middle[0]-index[0]);
        center_volume[0]=(ma*mb * (thumb[2]-middle[2]) + mb * (index[0]+thumb[0]) - ma * (index[0]+middle[0]))/(2*(mb-ma));
        center_volume[1]=-1/ma * (center_volume[0] - (thumb[0]+index[0])/2) +(thumb[2]+index[2])/2;
        radius_volume=sqrt((center_volume[0]-thumb[0])*(center_volume[0]-thumb[0]) + (center_volume[1]-thumb[2])*(center_volume[1]-thumb[2]));

        cout<<"center "<<center_volume.toString()<<endl;
        cout<<"radius "<<radius_volume<<endl;
    }

    /*******************************************************************************/
    void volumeFilter(vector<Vector> &points)
    {
        vector<Vector> pointsTmp=points;
        pointsOut.clear();

        for (size_t i=0;i<pointsTmp.size(); i++)
        {
            Vector point=pointsTmp[i];
            if ((point[0]-center_volume[0])*(point[0]-center_volume[0]) + (point[2]-center_volume[1])*(point[2]-center_volume[1]) - (radius_volume+a_offset)*(radius_volume+b_offset )<0 )
                pointsOut.push_back(point);
        }
    }

    /*******************************************************************************/
    void graspableEllips(vector<Vector> &fingerPoses)
    {
        cout<<"debug 1"<<endl;
        Vector thumb, index, middle;
        thumb=fingerPoses[0];
        index=fingerPoses[1];
        middle=fingerPoses[2];
        a=abs(thumb[0]-index[0])/2.0;
        b=sqrt(thumb[2]*thumb[2]+thumb[0]*thumb[0]);
        double ma, mb, ca, cb;
        ma=(index[2]-thumb[2])/(index[0]-thumb[0]);
        mb=-1/ma;

        ca=index[2]- index[0]*ma;
        cb=0;

        center_ellips[0]=(ca - cb)/(mb-ma);
        center_ellips[1]=ma*center_ellips[0]+ca;
    }

    /*******************************************************************************/
    void ellipsFilter(vector<Vector> &points)
    {
        vector<Vector> pointsTmp=points;
        pointsOut.clear();

        for (size_t i=0;i<pointsTmp.size(); i++)
        {
            Vector point=pointsTmp[i];
            if ((point[0]-center_ellips[0])*(point[0]-center_ellips[0])*b*b + (point[2]-center_ellips[1])*(point[2]-center_ellips[1])*a*a - (a+a_offset)*(a+a_offset)*(b+b_offset)*(b+b_offset)<0 )
                pointsOut.push_back(point);
        }
    }
};
