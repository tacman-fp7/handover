/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

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


/*******************************************************************************/
class pointCloudExtraction : public RFModule, public PortReader
{
protected:
    vector<cv::Point> floodPoints;
    cv::Point seed;

    Mutex mutex;

    string module_name;
    string homeContextPath;
    string savename;
    string fileFormat;
    int fileCount;

    int downsampling;
    double spatial_distance;

    bool seedAuto;
    bool flood3d;
    bool saving;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;

    BufferedPort<Bottle> portPointsOut;
    BufferedPort<ImageOf<PixelRgb> > portDispOut;
    Port portSeedIn;

    RpcClient portSFM;
    RpcClient portSeg;
    RpcServer portRpc;

    /*******************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle data; data.read(connection);
        cout << " Read: " << data.toString() << endl;
        if (data.size()>=2)
        {
            LockGuard lg(mutex);
            cv::Point point(data.get(0).asInt(),data.get(1).asInt());
            seed = point;
        }

        return true;
    }

    /*******************************************************************************/
    bool clear_points()
    {
        floodPoints.clear();
        return true;
    }

    /*******************************************************************************/
    bool set_auto_seed(string &entry)
    {
        if (entry == "yes")
            seedAuto=true;
        else if (entry == "no")
            seedAuto=false;
        else
            return false;

        return true;
    }

public:

    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("in-hand-segmentation"), "Getting module name").asString();

        homeContextPath=rf.getHomeContextPath().c_str();
        savename=rf.check("savename", Value("cloud3D"), "Default file savename").asString();
        saving=rf.check("savingClouds", Value(false), "Toggle save clouds as file").asBool();
        fileFormat=rf.check("format", Value("off"), "Default file format").asString();

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<savename<<"N."<<fileFormat<<", with increasing numeration N"<< endl;
        fileCount=0;

        downsampling=std::max(1,rf.check("downsampling",Value(1)).asInt());
        spatial_distance=rf.check("spatial_distance",Value(0.005)).asDouble();
        seedAuto = rf.check("seedAuto", Value(true), "Automatic seed computation").asBool();

        cout<<"Opening ports"<<endl;

        portDispIn.open("/" + module_name + "/disp:i");
        portImgIn.open("/" + module_name + "/img:i");

        portPointsOut.open("/"+module_name+"/pnt:o");
        portDispOut.open("/"+module_name+"/disp:o");

        portSFM.open("/"+module_name+"/SFM:rpc");
        portSeg.open("/"+module_name+"/seg:rpc");
        portRpc.open("/"+module_name+"/rpc:i");

        cout<<"Ports opened"<<endl;

        portSeedIn.setReader(*this);
        attach(portRpc);

        seed.x = -1; seed.y = -1;

        flood3d=false;

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portImgIn.interrupt();

        portPointsOut.interrupt();
        portDispOut.interrupt();

        portSFM.interrupt();
        portSeg.interrupt();
        portRpc.interrupt();

        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        portDispIn.close();
        portImgIn.close();

        portPointsOut.close();
        portDispOut.close();

        portSFM.close();
        portSeg.close();
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

        LockGuard lg(mutex);

        ImageOf<PixelRgb> &imgDispOut=portDispOut.prepare();
        imgDispOut.resize(imgDispIn->width(),imgDispIn->height());

        cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());
        cv::Mat imgDispOutMat=cv::cvarrToMat((IplImage*)imgDispOut.getIplImage());
        cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);

        if (flood3d)
        {
            //clearRec();

            Bottle &bpoints=portPointsOut.prepare();
            vector<Vector> points;

            cout<<"Getting seed "<<endl;

            if (seedAuto)  // Autoseed overwrites present values of 'seed'
            {
                if(!getDepthSeed(imgDispInMat,seed))
                {
                    portPointsOut.unprepare();
                    portDispOut.write();
                    return true;
                }
            }
            else
            // Wait for click only if seed is not auto and coords have not been given by command or on a previous click.

            if ((seed.x<0) && (seed.y<0))
            {
                cout << " click for a seed" << endl;
                portPointsOut.unprepare();
                portDispOut.write();
                return true;
            }
            // If none of the conditions apply, means seed was either given by command or clicked before.

            cout << "3D points flood3D "<<endl;

            // Use the seed point to get points from SFM with the Flood3D command, and spatial_distance given
            Bottle cmdSFM,replySFM;
            cmdSFM.addString("Flood3D");
            cmdSFM.addInt(seed.x);
            cmdSFM.addInt(seed.y);
            cmdSFM.addDouble(spatial_distance);
            bool ok = portSFM.write(cmdSFM,replySFM);
            if (ok)
            {
                for (int i=0; i<replySFM.size(); i+=5)
                {
                    int x=replySFM.get(i+0).asInt();
                    int y=replySFM.get(i+1).asInt();
                    PixelRgb px=imgIn->pixel(x,y);

                    Vector point(6,0.0);
                    point[0]=replySFM.get(i+2).asDouble();
                    point[1]=replySFM.get(i+3).asDouble();
                    point[2]=replySFM.get(i+4).asDouble();
                    point[3]=px.r;
                    point[4]=px.g;
                    point[5]=px.b;

                    points.push_back(point);
                    floodPoints.push_back(cv::Point(x,y));

                    Bottle &bpoint = bpoints.addList();
                    bpoint.addDouble(point[0]);
                    bpoint.addDouble(point[1]);
                    bpoint.addDouble(point[2]);
                }

               cout << "Retrieved " << points.size() << " 3D points" <<endl;
            }
            else
            {
                cout << " SFM didn't reply" << endl;
                return true;
            }

            if (points.size()>0)
            {
                if (saving)
                {
                    saveCloud(points);
                }

                portPointsOut.write();
            }
            else
            {
                portPointsOut.unprepare();
            }

            seed.x = -1; seed.y = -1; // Reset seed after each rec call

            flood3d=false;
            points.clear();
            bpoints.clear();

            PixelRgb color(255,255,0);
            for (size_t i=0; i<floodPoints.size(); i++)
                imgDispOut.pixel(floodPoints[i].x,floodPoints[i].y)=color;

            portDispOut.write();
            return true;
        }

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
    }

    /*******************************************************************************/
    bool getDepthSeed(cv::Point2i &seedPoint)
    {
        ImageOf<PixelMono> *imgDispIn=portDispIn.read();
        if (imgDispIn==NULL)
            return false;

        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());

        return getDepthSeed(imgDispInMat,seedPoint);
    }

    /*******************************************************************************/
    bool getDepthSeed(const cv::Mat &disparity,cv::Point2i &seedPoint)
    {
        cout << "Finding seed automatically" << endl;
        cv::Mat depth = disparity.clone();

        /* Filter */
        int gaussSize = 5;
        double sigmaX1 = 1.5;
        double sigmaY1 = 1.5;
        cv::GaussianBlur(depth, depth, cv::Size(gaussSize,gaussSize), sigmaX1, sigmaY1);

        cv::threshold(depth, depth, 30, -1, CV_THRESH_TOZERO);

        int dilate_niter = 4;
        int erode_niter = 2;
        double sigmaX2 = 2;
        double sigmaY2 = 2;

        cv::dilate(depth, depth, cv::Mat(), cv::Point(-1,-1), dilate_niter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        cv::GaussianBlur(depth, depth, cv::Size(gaussSize,gaussSize), sigmaX2, sigmaY2, cv::BORDER_DEFAULT);

        cv::erode(depth, depth, cv::Mat(), cv::Point(-1,-1), erode_niter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        /* Find closest valid blob */
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;

        int fillFlags = 8 | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE; // flags for floodFill

        cv::Mat aux = depth.clone();

        int minBlobSize = 300;
        int fillSize = 0;
        int imageThreshRatioLow = 10;
        int imageThreshRatioHigh = 20;
        while (fillSize < minBlobSize)
        {
            cv::minMaxLoc( aux, &minVal, &maxVal, &minLoc, &maxLoc );

            // if its too small, paint it black and search again
            fillSize = floodFill(aux, maxLoc, 0, 0, cv::Scalar(maxVal/imageThreshRatioLow), cv::Scalar(maxVal/imageThreshRatioHigh), fillFlags);
        }
        // paint closest valid blob white
        cv::Mat fillMask(depth.rows+2, depth.cols+2, CV_8U);
        fillMask.setTo(0);
        cv::floodFill(depth, fillMask, maxLoc, 255, 0, cv::Scalar(maxVal/imageThreshRatioLow), cv::Scalar(maxVal/imageThreshRatioHigh), cv::FLOODFILL_MASK_ONLY + fillFlags);

        /* Find contours */
        std::vector<std::vector<cv::Point > > contours;
        std::vector<cv::Vec4i> hierarchy;

        // use aux because findContours modifies the input image
        aux = fillMask(cv::Range(1,depth.rows+1), cv::Range(1,depth.cols+1)).clone();
        cv::findContours( aux, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /* If any blob is found check again that only the biggest valid blob is selected */
        int blobI = -1;
        double blobSizeOld = -1, blobSize = -1;
        for( int c = 0; c < contours.size(); c++ )
        {
            // find the area of contour
            blobSize = cv::contourArea(contours[c]);

            // select only the biggest valid blob
            if( blobSize > minBlobSize && blobSize > blobSizeOld)
            {
                blobI = c;
                blobSizeOld = blobSize;
            }
        }

        /* If any blob is found (after the double-check) */
        if (blobI>=0)
        {
            /* Get the Bounding Box */
            cv::Rect blobBox = cv::boundingRect(contours[blobI]);

            double seedValid = -1;      // false
            cv::Point2i seedAux;
            seedAux.x = blobBox.tl().x;
            seedAux.y = blobBox.tl().y;
            while (seedValid < 0)
            {              // Move diagonally through the image until a point inside the blob is found.
                seedAux.x = seedAux.x + 4;
                seedAux.y = seedAux.y + 1;

                cv::circle(depth, seedAux, 1, cv::Scalar(0,0,0),2);

                if ((seedAux.x > blobBox.br().x ) ||(seedAux.y > blobBox.br().y )){
                    cout << "Seed could not be found"<< endl;
                    return false;
                }

                seedValid = pointPolygonTest(contours[blobI], seedAux, false );
            }

            seedPoint.x = seedAux.x+2;     // add a small margin to be fully inside the contour
            seedPoint.y = seedAux.y+2;

            cout << "Seed found at " << seedPoint.x << " ," << seedPoint.y << endl;

            return true;
        }
        else
        {
            cout << "Seed could not be determined " << endl;
            return false;
        }
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

    pointCloudExtraction mod;
    ResourceFinder rf;
    rf.setDefaultContext("seg2cloud");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
