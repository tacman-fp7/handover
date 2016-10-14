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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class filteringModule : public RFModule
{
    bool online;
    string module_name;
    string homeContextPath;
    string savename;
    string fileOutFormat;
    string fileInFormat;
    string fileInName;

    bool saving;
    int fileCount;
    int down;

    BufferedPort<Bottle> portPointsIn;
    BufferedPort<Bottle> portPointsOut;
    RpcServer portRpc;

    vector<Vector> pointsIn;
    vector<Vector> pointsOut;

    double radius;
    int nnThreshold;
    int numVertices;

    bool spatial_filter;

public:

    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("cloudFiltering"), "Getting module name").asString();

        online=(rf.check("online", Value("no")).asString()=="yes");

        homeContextPath=rf.getHomeContextPath().c_str();
        savename=rf.check("savename", Value("filtered-cloud"), "Default file savename").asString();
        saving=(rf.check("savingClouds", Value("yes"), "Toggle save clouds as file").asString()=="yes");
        fileOutFormat=rf.check("format", Value("off"), "Default file format").asString();
        down= rf.check("downsampling", Value(1)).asInt();
        fileInFormat="off"; // to be extended

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<savename<<"N."<<fileOutFormat<<", with increasing numeration N"<< endl;
        fileCount=0;

        spatial_filter=(rf.check("spatial_filter", Value("yes")).asString()=="yes");
        radius=rf.check("radius", Value(0.0002)).asDouble();
        nnThreshold=rf.check("nn-threshold", Value(40)).asInt();

        if (online)
        {
            portPointsIn.open("/" + module_name + "pnt:i");
            portPointsOut.open("/" + module_name + "pnt:o");
        }
        else
        {
            fileInName=rf.check("fileInName", Value("cloud.off"), "Default cloud name").asString();
            cout<<"fileInName "<<fileInName<<endl;
            return readPointCloud();
        }

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        return true;
    }

    /*******************************************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /*******************************************************************************/
    bool updateModule()
    {
        if (online)
            waitForCloud();

        if (spatial_filter)
            spatialDensityFilter(radius,nnThreshold+1, pointsIn);

        Vector colors(3,0.0);
        colors[1]=255;

        saveNewCloud(colors);

        return false;
    }

    /*******************************************************************************/
    bool readPointCloud()
    {
        int state=0;
        int nPoints;
        char line[255];
        deque<Vector> points_tmp;
        Vector point_tmp;
        point_tmp.resize(6,0.0);

        cout<< "In cloud file "<<homeContextPath+"/"+fileInName<<endl;

        ifstream cloudFile((homeContextPath+"/"+fileInName).c_str());
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
                if (tmp=="COFF")
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
                            cout<<"point "<<point_tmp.toString(3,3).c_str()<<endl;
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
    bool waitForCloud()
    {
        return true;
    }

    /*******************************************************************************/
    vector<int>  spatialDensityFilter(const double radius, const int maxResults, vector<Vector> &pointsIn)
    {
        numVertices=pointsIn.size();

        cv:: Mat data(numVertices,3,CV_32F);
        for (int i=0; i<numVertices; i++)
        {
            Vector point=pointsIn[i];
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

            res[i]=kdtree.radiusSearch(query,indices,dists,radius,maxResults,cv::flann::SearchParams(128));

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
    bool saveNewCloud(const Vector &colors)
    {
        ofstream fout;
        stringstream fileName;
        string fileNameFormat;
        fileName.str("");
        fileName << homeContextPath + "/" + savename.c_str() << fileCount;

        if (fileOutFormat == "ply")
        {
            fileNameFormat = fileName.str()+".ply";
            cout << "Saving as " << fileNameFormat << endl;
            fout.open(fileNameFormat.c_str());
            if (fout.is_open())
            {
                fout << "ply\n";
                fout << "format ascii 1.0\n";
                fout << "element vertex " << pointsOut.size() <<"\n";
                fout << "property float x\n";
                fout << "property float y\n";
                fout << "property float z\n";
                fout << "property uchar diffuse_red\n";
                fout << "property uchar diffuse_green\n";
                fout << "property uchar diffuse_blue\n";
                fout << "end_header\n";

                for (unsigned int i=0; i<pointsOut.size(); i++)
                {
                    fout << pointsOut[i][0] << " " <<      pointsOut[i][1] << " " <<      pointsOut[i][2] << " " << (int)pointsOut[i][3] << " " << (int)pointsOut[i][4] << " " << (int)pointsOut[i][5] << "\n";
                    //plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";
                }

                fout.close();
                cout << "Points saved as " << fileNameFormat << endl;
                fileCount++;
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
                fout<<pointsOut.size()<<" 0 0"<<endl;
                fout<<endl;
                for (size_t i=0; i<pointsOut.size(); i++)
                {
                    fout<<pointsOut[i].subVector(0,2).toString(3,4).c_str()<<" "<<
                          colors[0]<<" "<<colors[1]<<" "<<colors[2]<<endl;
                }
                fout<<endl;
            }

            fout.close();
            cout << "Points saved as " << fileNameFormat << endl;
            fileCount++;
        }
        else if (fileOutFormat == "none")
        {
            cout << "Points not saved" << endl;
        }
    }

};
