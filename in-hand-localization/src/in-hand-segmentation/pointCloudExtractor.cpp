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

#include "pointCloudExtraction_IDL.h"

class pointCloudExtraction : public RFModule,
                             public pointCloudExtraction_IDL
{
protected:
    vector<cv::Point> blobPoints;
    vector<Vector> points;

    Mutex mutex;

    string module_name;
    string homeContextPath;
    string savename;
    string fileFormat;
    int fileCount;
    bool saving;
    bool acquire;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelMono> > portBlobIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<Bottle> portPointIn;

    BufferedPort<ImageOf<PixelRgb> > portDispOut;

    RpcClient portSFM;
    RpcServer portRpc;

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /*******************************************************************************/
    bool clear_points()
    {
        points.clear();
        return true;
    }

    /*******************************************************************************/
    bool set_saving(const string &entry)
    {
        if (entry == "yes")
            saving=true;
        else if (entry == "no")
            saving=false;
        else
            return false;

        return true;
    }

    /*******************************************************************************/
    bool set_format(const string &entry)
    {
        if (entry == "off" || entry == "ply")
        {
            fileFormat=entry;

            return true;
        }
        else
        {
            return false;
        }
    }

    /*******************************************************************************/
    bool set_filename(const string &entry)
    {
        savename=entry;
        return true;
    }

    /*******************************************************************************/
    bool acquiring(const string &entry)
    {
        if (entry=="yes")
            acquire=true;
        else
            acquire=false;

        return true;
    }

    /*******************************************************************************/
    Bottle get_2D_blob_points()
    {
        Bottle bblobs;

        if (blobPoints.size()>0)
        {
            for (size_t i=0; i<blobPoints.size(); i++)
            {
                Bottle &bblob=bblobs.addList();
                bblob.addInt(blobPoints[i].x); bblob.addInt(blobPoints[i].y);
            }
        }

        return bblobs;
    }

    /*******************************************************************************/
    Bottle get_3D_blob_points()
    {
        Bottle bpoints;
        if (points.size()>0)
        {
            for (size_t i=0; i<points.size(); i++)
            {
                Vector point=points[i];
                Bottle &bpoint=bpoints.addList();
                bpoint.addDouble(point[0]); bpoint.addDouble(point[1]);bpoint.addDouble(point[2]);
            }
        }

        return bpoints;
    }

public:
    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("cloud-extractor"), "Getting module name").asString();

        homeContextPath=rf.getHomeContextPath().c_str();
        savename=rf.check("savename", Value("cloud3D"), "Default file savename").asString();
        saving=(rf.check("savingClouds", Value("yes"), "Toggle save clouds as file").asString()== "yes");
        fileFormat=rf.check("format", Value("off"), "Default file format").asString();

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<savename<<"N."<<fileFormat<<", with increasing numeration N"<< endl;
        fileCount=0;

        cout<<"Opening ports"<<endl;

        portDispIn.open("/" + module_name + "/disp:i");
        portImgIn.open("/" + module_name + "/img:i");
        portBlobIn.open("/" + module_name + "/blob:i");

        portDispOut.open("/"+module_name+"/disp:o");

        portSFM.open("/"+module_name+"/SFM:rpc");
        portRpc.open("/"+module_name+"/rpc");

        cout<<"Ports opened"<<endl;

        cout<<"attach "<< attach(portRpc)<<endl;

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portImgIn.interrupt();
        portBlobIn.interrupt();
        portPointIn.interrupt();

        portDispOut.interrupt();

        portSFM.interrupt();
        portRpc.interrupt();

        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        portDispIn.close();
        portImgIn.close();
        portBlobIn.close();
        portPointIn.close();

        portDispOut.close();

        portSFM.close();
        portRpc.close();

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
        ImageOf<PixelMono> *imgDispIn=portDispIn.read();
        if (imgDispIn==NULL)
            return false;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelMono> *imgBlobIn=portBlobIn.read();
        if (imgBlobIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgDispOut=portDispOut.prepare();
        imgDispOut.resize(imgDispIn->width(),imgDispIn->height());

        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());
        cv::Mat imgDispOutMat=cv::cvarrToMat((IplImage*)imgDispOut.getIplImage());
        cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);

        cv::Mat imgBlobInMat=cv::cvarrToMat((IplImage*)imgBlobIn->getIplImage());

        if (acquire)
        {
            blobPoints.clear();
            points.clear();
      
            for (int j=0; j<imgBlobInMat.rows; j++)
            {
                for (int i=0; i<imgBlobInMat.cols; i++)
                {
                    if (static_cast<int>(imgBlobInMat.at<unsigned char>(j,i))!=0)
                    {   
                        blobPoints.push_back(cv::Point(j,i));                       
                    }
                }
            }

            cout<<"Number of points belonging to the blob: "<<blobPoints.size()<<endl;

            LockGuard lg(mutex);

            // Use the seed point to get points from SFM with the Flood3D command, and spatial_distance given
            Bottle cmdSFM,replySFM;
            cmdSFM.addString("Points");
            int count=0;

            if (blobPoints.size()>0)
            {              
                for (size_t i=0; i<blobPoints.size(); i++)
                {
                    //if ((blobPoints[i].y<320) && (blobPoints[i].x<240) && (blobPoints[i].x>0) && (blobPoints[i].y>0))
                    //{
                        cv::Point single_point=blobPoints[i];
                        cmdSFM.addInt(single_point.y);
                        cmdSFM.addInt(single_point.x);
                    //}
                }

                if (portSFM.write(cmdSFM,replySFM))
                {
                    for (int i=0; i<replySFM.size(); i+=3)
                    {
                        Vector point(6,0.0);
                        point[0]=replySFM.get(i+0).asDouble();
                        point[1]=replySFM.get(i+1).asDouble();
                        point[2]=replySFM.get(i+2).asDouble();

                        PixelRgb px=imgIn->pixel(blobPoints[count].y,blobPoints[count].x);
                        point[3]=px.r;
                        point[4]=px.g;
                        point[5]=px.b;

                        count++;

                        if ((norm(point)>0))
                        {
                            points.push_back(point);
                        }
                        }

                    cout << "Retrieved " << points.size() << " 3D points" <<endl;
                }
                else
                {
                    cout << " SFM didn't reply!" << endl;
                    return true;
                }
            }
            else
                yError()<<"No blob received!";
        }

        if (points.size()>0 && acquire==true)
        {
            if (saving)
            {
                saveCloud(points);
                acquire=false;
            }
        }

        if (blobPoints.size()>0)
        {            
            PixelRgb color(255,255,0);
            for (size_t i=0; i<blobPoints.size(); i++)
            {
                if ((blobPoints[i].y<320) && (blobPoints[i].x<240) && (blobPoints[i].x>0) && (blobPoints[i].y>0))
                    imgDispOut.pixel(blobPoints[i].y,blobPoints[i].x)=color;
            }
        }

        portDispOut.write();

        return true;
    }

    /*******************************************************************************/
    bool saveCloud(const vector<Vector> &points)
    {
        ofstream fout;
        stringstream fileName;
        string fileNameFormat;
        fileName.str("");
        fileName << homeContextPath + "/" + savename.c_str() << fileCount;

        if (fileFormat == "ply")
        {
            fileNameFormat = fileName.str()+".ply";
            cout << "Saving as " << fileNameFormat << endl;
            fout.open(fileNameFormat.c_str());
            if (fout.is_open())
            {
                fout << "ply\n";
                fout << "format ascii 1.0\n";
                fout << "element vertex " << points.size() <<"\n";
                fout << "property float x\n";
                fout << "property float y\n";
                fout << "property float z\n";
                fout << "property uchar diffuse_red\n";
                fout << "property uchar diffuse_green\n";
                fout << "property uchar diffuse_blue\n";
                fout << "end_header\n";

                for (unsigned int i=0; i<points.size(); i++)
                {
                    fout << points[i][0] << " " <<      points[i][1] << " " <<      points[i][2] << " " << (int)points[i][3] << " " << (int)points[i][4] << " " << (int)points[i][5] << "\n";
                    //plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";
                }

                fout.close();
                cout << "Points saved as " << fileNameFormat << endl;
                fileCount++;
            }

        }
        else if (fileFormat == "off")
        {
            fileNameFormat = fileName.str()+".off";
            cout << "Saving as " << fileNameFormat << endl;
            fout.open(fileNameFormat.c_str());
            if (fout.is_open())
            {

                fout<<"COFF"<<endl;
                fout<<points.size()<<" 0 0"<<endl;
                fout<<endl;
                for (size_t i=0; i<points.size(); i++)
                {
                    fout<<points[i].subVector(0,2).toString(3,4).c_str()<<" "<<
                          points[i].subVector(3,5).toString(0,3).c_str()<<endl;
                }
                fout<<endl;
            }

            fout.close();
            cout << "Points saved as " << fileNameFormat << endl;
            fileCount++;
        }
        else if (fileFormat == "none")
        {
            cout << "Points not saved" << endl;
        }
        return true;
    }
};
