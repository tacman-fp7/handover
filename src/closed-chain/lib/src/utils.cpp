#include "iCub/periPersonalSpace/utils.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;
using namespace iCub::skinDynLib;

using namespace std;

void closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void matrixOfIntIntoBottle(const yarp::sig::Matrix m, Bottle &b)
{
    Vector v = toVector(m);

    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addInt(int(v[i]));
    }
}

string int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

unsigned int factorial(unsigned int n)
{
    if (n == 0)
       return 1;
    return n * factorial(n - 1);
}

/****************************************************************/
/* INCOMING EVENT WRAPPER
*****************************************************************/
    IncomingEvent::IncomingEvent()
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;
        Threat = 0.0;
    }

    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
        Threat = 0.0;
    }

    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const double threat, const std::string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
        Threat = threat;
    }

    IncomingEvent::IncomingEvent(const IncomingEvent &e)
    {
        *this = e;
    }

    IncomingEvent::IncomingEvent(const Bottle &b)
    {
        fromBottle(b);
    }

    IncomingEvent & IncomingEvent::operator=(const IncomingEvent &e)
    {
        Pos    = e.Pos;
        Vel    = e.Vel;
        Src    = e.Src;
        Radius = e.Radius;
        Threat = e.Threat;
        return *this;
    }

    Bottle IncomingEvent::toBottle()
    {
        Bottle b;
        b.clear();

        b.addDouble(Pos[0]);
        b.addDouble(Pos[1]);
        b.addDouble(Pos[2]);
        b.addDouble(Vel[0]);
        b.addDouble(Vel[1]);
        b.addDouble(Vel[2]);
        b.addDouble(Radius);
        b.addString(Src);
        b.addDouble(Threat); //keep it last for now - for backward compatibility

        return b;
    }

    bool IncomingEvent::fromBottle(const Bottle &b)
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;
        Threat = 0.0;

        Pos[0] = b.get(0).asDouble();
        Pos[1] = b.get(1).asDouble();
        Pos[2] = b.get(2).asDouble();

        Vel[0] = b.get(3).asDouble();
        Vel[1] = b.get(4).asDouble();
        Vel[2] = b.get(5).asDouble();

        Radius = b.get(6).asDouble();
        Src    = b.get(7).asString();
        Threat = b.get(8).asDouble();

        return true;
    }

    void IncomingEvent::print()
    {
        yDebug("\tPos: %s\t Vel: %s\t Radius %g\t Src %s Threat: %g\n",Pos.toString().c_str(),Vel.toString().c_str(),Radius,Src.c_str(),Threat);
    }

    string IncomingEvent::toString() const
    {
        stringstream res;
        res << "Pos: "<< Pos.toString(3,3) << " Vel: "<< Vel.toString(3,3)
            << " Radius:"<< Radius << " Src:"<< Src << " threat:"<< Threat ;
        return res.str();
    }

/****************************************************************/
/* INCOMING EVENT 4 TAXEL WRAPPER
*****************************************************************/
    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE() : IncomingEvent()
    {
        NRM = 0;
        TTC = 0;
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const Vector &p, const Vector &v,
                                                   const double r, const string &s) :
                                                   IncomingEvent(p,v,r,s)
    {
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e)
    {
        *this = e;
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent &e)
    {
        *this = e;
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent4TaxelPWE &e)
    {
        IncomingEvent::operator=(e);
        TTC    = e.TTC;
        NRM    = e.NRM;
        return *this;
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent &e)
    {
        IncomingEvent::operator=(e);
        computeNRMTTC();
        return *this;
    }

    void IncomingEvent4TaxelPWE::computeNRMTTC()
    {
        int sgn = Pos[2]>=0?1:-1;
        NRM = sgn * norm(Pos);

        if (dot(Pos,Vel)==0)
            TTC = 0;
        else
            TTC = -norm(Pos)*norm(Pos)/dot(Pos,Vel);
    }

    std::vector<double> IncomingEvent4TaxelPWE::getNRMTTC()
    {
        std::vector<double> x(2);
        x[0] = NRM;
        x[1] = TTC;

        return x;
    }
    
    double IncomingEvent4TaxelPWE::getNRM()
    {
        return NRM;
    }

    double IncomingEvent4TaxelPWE::getTTC()
    {
        return TTC;
    }
    
    void IncomingEvent4TaxelPWE::print()
    {
        yDebug("\tNRM: %g\t TTC: %g \t %s", NRM, TTC, IncomingEvent::toString().c_str());
    }

    string IncomingEvent4TaxelPWE::toString() const
    {
        stringstream res;
        res << setprecision(3) << "NRM:"<< NRM << " TTC:" << TTC << " "<< IncomingEvent::toString();
        return res.str();
    }

/****************************************************************/
/* EYE WRAPPER
*****************************************************************/
    bool getAlignHN(const ResourceFinder &rf, const string &type,
                    iKinChain *chain, const bool verbose)
    {
        ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
        if ((chain!=NULL) && _rf.isConfigured())
        {
            string message=_rf.findFile("from").c_str();
            if (!message.empty())
            {
                message+=": aligning matrix for "+type;
                Bottle &parType=_rf.findGroup(type.c_str());
                if (!parType.isNull())
                {
                    if (Bottle *bH=parType.find("HN").asList())
                    {
                        int i=0;
                        int j=0;

                        Matrix HN(4,4); HN=0.0;
                        for (int cnt=0; (cnt<bH->size()) && (cnt<HN.rows()*HN.cols()); cnt++)
                        {
                            HN(i,j)=bH->get(cnt).asDouble();
                            if (++j>=HN.cols())
                            {
                                i++;
                                j=0;
                            }
                        }

                        // enforce the homogeneous property
                        HN(3,0)=HN(3,1)=HN(3,2)=0.0;
                        HN(3,3)=1.0;

                        chain->setHN(HN);

                        if (verbose)
                        {
                            fprintf(stdout,"%s found:\n",message.c_str());
                            fprintf(stdout,"%s\n",HN.toString(3,3).c_str());
                        }

                        return true;
                    }
                }
            }
            else
            {
                message=_rf.find("from").asString().c_str();
                message+=": aligning matrix for "+type;
            }
            if (verbose)
                fprintf(stdout,"%s not found!\n",message.c_str());
        }

        return false;
    };

    /************************************************************************/
    bool getCamPrj(const ResourceFinder &rf, const string &type,
                   Matrix **Prj, const bool verbose)
    {
        ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
        *Prj=NULL;

        if (!_rf.isConfigured())
            return false;

        string message=_rf.findFile("from").c_str();
        if (!message.empty())
        {
            message+=": intrinsic parameters for "+type;
            Bottle &parType=_rf.findGroup(type.c_str());
            if (!parType.isNull())
            {
                if (parType.check("fx") && parType.check("fy") &&
                    parType.check("cx") && parType.check("cy"))
                {
                    double fx=parType.find("fx").asDouble();
                    double fy=parType.find("fy").asDouble();
                    double cx=parType.find("cx").asDouble();
                    double cy=parType.find("cy").asDouble();

                    if (verbose)
                    {
                        fprintf(stdout,"%s found:\n",message.c_str());
                        fprintf(stdout,"fx = %g\n",fx);
                        fprintf(stdout,"fy = %g\n",fy);
                        fprintf(stdout,"cx = %g\n",cx);
                        fprintf(stdout,"cy = %g\n",cy);
                    }

                    Matrix K=eye(3,3);
                    Matrix Pi=zeros(3,4);

                    K(0,0)=fx; K(1,1)=fy;
                    K(0,2)=cx; K(1,2)=cy;

                    Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0;

                    *Prj=new Matrix;
                    **Prj=K*Pi;

                    return true;
                }
            }
        }
        else
        {
            message=_rf.find("from").asString().c_str();
            message+=": intrinsic parameters for "+type;
        }

        if (verbose)
            fprintf(stdout,"%s not found!\n",message.c_str());

        return false;
    }

    /************************************************************************/
    eyeWrapper::eyeWrapper(string _name, double _hV, const ResourceFinder &_eyeCalibRF) :
               name(_name), headVersion(_hV)
    {
        if (name=="left")
        {
            eye=new iCubEye(headVersion>1.0?"left_v2":"left");
        }
        else if (name=="right")
        {
            eye=new iCubEye(headVersion>1.0?"right_v2":"right");
        }

        // remove constraints on the links
        // we use the chains for logging purpose
        eye->setAllConstraints(false);

        // release links
        eye->releaseLink(0);
        eye->releaseLink(1);
        eye->releaseLink(2);

        // add aligning matrices read from configuration file
        if (name=="left")
        {
            getAlignHN(_eyeCalibRF,"ALIGN_KIN_LEFT",eye->asChain(),true);
        }
        else if (name=="right")
        {
            getAlignHN(_eyeCalibRF,"ALIGN_KIN_RIGHT",eye->asChain(),true);
        }

        bool ret=0;

        // get camera projection matrix
        if (name=="left")
        {
            ret=getCamPrj(_eyeCalibRF,"CAMERA_CALIBRATION_LEFT",&Prj,true);
        }
        else if (name=="right")
        {
            ret=getCamPrj(_eyeCalibRF,"CAMERA_CALIBRATION_RIGHT",&Prj,true);
        }

        if (!ret)
            Prj=NULL;
    }

    eyeWrapper & eyeWrapper::operator= (const eyeWrapper &ew)
    {
        name        = ew.name;
        eye         = ew.eye;
        headVersion = ew.headVersion;
        Prj         = ew.Prj;
        return *this;
    }

// empty line to make gcc happy
