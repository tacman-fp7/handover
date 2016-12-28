#include "iCub/periPersonalSpace/taxelPWE.h"

using namespace  yarp::os;
using namespace yarp::sig;
using namespace       std;

#define TAXEL_RF_ANGLE_DEG 50.0
#define DESIRED_RADIUS_XY_AT_RF_APEX 0.07 // meters; we don't want the RF spherical sector to start at the apex,
//but we want to truncate it such that it starts at the height with this radius;

/****************************************************************/
/* TAXEL WRAPPER FOR PWE
*****************************************************************/

    TaxelPWE::TaxelPWE() : Taxel()
    {
        Resp    = 0.0;
        RFangle = TAXEL_RF_ANGLE_DEG*M_PI/180.0;
        sphericalSectorShiftOffset = DESIRED_RADIUS_XY_AT_RF_APEX / tan(TAXEL_RF_ANGLE_DEG*M_PI/180.0); 
        //With a cone having the apex at the taxel, we calculate the height at which it has a desired radius
        //This will be the RF offset
    }

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n) : Taxel(p,n)
    {
        Resp    = 0.0;
        RFangle = TAXEL_RF_ANGLE_DEG*M_PI/180.0;
        sphericalSectorShiftOffset = DESIRED_RADIUS_XY_AT_RF_APEX / tan(TAXEL_RF_ANGLE_DEG*M_PI/180.0); 
        //With a cone having the apex at the taxel, we calculate the height at which it has a desired radius
    };

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n,
                       const int &i) : Taxel(p,n,i)
    {
        Resp    = 0.0;
        RFangle = TAXEL_RF_ANGLE_DEG*M_PI/180.0;
        sphericalSectorShiftOffset = DESIRED_RADIUS_XY_AT_RF_APEX / tan(TAXEL_RF_ANGLE_DEG*M_PI/180.0); 
        //With a cone having the apex at the taxel, we calculate the height at which it has a desired radius
    };

    bool TaxelPWE::addSample(IncomingEvent4TaxelPWE ie)
    {
        //check for inside RF is called outside
        //if (!insideRFCheck(ie)){ //should be now redundant and can be removed - is checked before
        //    printf("Warning: TaxelPWE::addSample() - called with event outside of taxel RF.\n");
        //    return false;
        //}

        std::vector <double> x = ie.getNRMTTC();
        // printf("[TaxelPWE::addSample] x %g %g\n",x[0],x[1]);

        return pwe->addSample(x);
    }

    bool TaxelPWE::removeSample(IncomingEvent4TaxelPWE ie)
    {
        //check for inside RF is called outside
        //if (!insideRFCheck(ie)){ //should be now redundant and can be removed -  - is checked before
        //    printf("Warning: TaxelPWE::removeSample() - called with event outside of taxel RF.\n");
        //    return false;
        //}
        
        std::vector <double> x = ie.getNRMTTC();
        return pwe->removeSample(x);
    }

    bool TaxelPWE::insideRFCheck(const IncomingEvent4TaxelPWE ie)
    {
        double distanceSquared; //squared Euclidean distance of stimulus from taxel 
        //the assumption is that max extension is >=0 and min extension is <=0 
        if(ie.Pos(2)>0)  //stimulus with positive z (along taxel normal)
        {        
            if(ie.Pos(2) <= (pwe->getExt())(0,1) ) //is z-coordinate within limit? if not, it surely is not inside the spherical sector 
            {
                //printf("[TaxelPWE::insideRFCheck]: positive z-coordinate %.3f inside limit (%.3f)\n",ie.Pos(2),(pwe->getExt())(0,1));
                Vector sphericalSectorCenter(3,0.0); 
                sphericalSectorCenter(2) = -sphericalSectorShiftOffset;
                double maxRadiusZ = (pwe->getExt())(0,1) + sphericalSectorShiftOffset; //max radius from shifted origin of sph. sector
                distanceSquared = pow(ie.Pos(0)-sphericalSectorCenter(0),2) + pow(ie.Pos(1)-sphericalSectorCenter(1),2) + pow(ie.Pos(2)- sphericalSectorCenter(2),2); //checking distance for full sphere, prior to considering the sector 
                if (distanceSquared <= maxRadiusZ * maxRadiusZ)
                {
                    //printf("[TaxelPWE::insideRFCheck]: distanceSquared: %.4f <= maxRadiusZ^2: %.4f\n",distanceSquared,maxRadiusZ*maxRadiusZ);
                    double a = tan(RFangle) * (sphericalSectorShiftOffset + ie.Pos(2)); //radius of RF sector at specific height 
                    if( (abs(ie.Pos(0)) <= a) && (abs(ie.Pos(1)) <= a) ) //stimulus x and y is within the sector at that height
                    {
                        //printf("[TaxelPWE::insideRFCheck]: Both, x and y coordinates (%.3f,%.3f) are inside the radius a %.3f.\n",ie.Pos(0),ie.Pos(1),a);
                        return true;
                    }
                    else
                    {
                        //printf("[TaxelPWE::insideRFCheck]: At least one of, x and y coordinates (%.3f,%.3f) is outside the radius a %.3f.\n",ie.Pos(0),ie.Pos(1),a);
                        return false;
                    }
                }
                else
                {
                   // printf("[TaxelPWE::insideRFCheck]: distanceSquared: %.4f > maxRadiusZ^2: %.4f\n",distanceSquared,maxRadiusZ*maxRadiusZ);
                    return false;
                }
            }
            else
            {
                //printf("[TaxelPWE::insideRFCheck]: positive z-coordinate %.3f outside limit (%.3f)\n",ie.Pos(2),(pwe->getExt())(0,1));
                return false; 
            }
        }
        else  //stimulus with negative z (along taxel normal) according to coordinate transform pipeline 
        {
            if(ie.Pos(2) >= (pwe->getExt())(0,0) ) //is z-coordinate within limit? if not, it surely is not inside the spherical sector 
            {
                //printf("[TaxelPWE::insideRFCheck]: negative z-coordinate %.3f inside limit (%.3f)\n",ie.Pos(2),(pwe->getExt())(0,0));
                Vector sphericalSectorNegativeCenter(3,0.0); 
                sphericalSectorNegativeCenter(2) = sphericalSectorShiftOffset;
                double maxRadiusNegZ = abs((pwe->getExt())(0,0)) + sphericalSectorShiftOffset; //max radius from shifted origin of sph. sector
                distanceSquared = pow(ie.Pos(0)-sphericalSectorNegativeCenter(0),2) + pow(ie.Pos(1)-sphericalSectorNegativeCenter(1),2) + pow(ie.Pos(2)-sphericalSectorNegativeCenter(2),2);  
                //checking distance for full sphere, prior to considering the sector
                if (distanceSquared <= maxRadiusNegZ * maxRadiusNegZ)
                {
                    //printf("[TaxelPWE::insideRFCheck]: distanceSquared: %.4f <= maxRadiusNegZ^2: %.4f\n",distanceSquared,maxRadiusNegZ*maxRadiusNegZ);
                    double a = tan(RFangle) * (sphericalSectorShiftOffset + abs(ie.Pos(2))); //radius of RF sector at specific height 
                    if( (abs(ie.Pos(0)) <= a) && (abs(ie.Pos(1)) <= a) ) //stimulus x and y is within the sector at that height
                    {
                       // printf("[TaxelPWE::insideRFCheck]: Both, x and y coordinates (%.3f,%.3f) are inside the radius a %.3f.\n",ie.Pos(0),ie.Pos(1),a);
                        return true;
                    }
                    else
                    {
                       //printf("[TaxelPWE::insideRFCheck]: At least one of, x and y coordinates (%.3f,%.3f) is outside the radius a %.3f.\n",ie.Pos(0),ie.Pos(1),a);
                        return false;
                    }
                }
                else
                {
                    //printf("[TaxelPWE::insideRFCheck]: distanceSquared: %.4f > maxRadiusNegZ^2: %.4f\n",distanceSquared,maxRadiusNegZ*maxRadiusNegZ);
                    return false;
                }
            }
            else
            {
               // printf("[TaxelPWE::insideRFCheck]: negative z-coordinate %.3f outside limit (%.3f)\n",ie.Pos(2),(pwe->getExt())(0,0));
                return false; 
            }
        }
    }

    void TaxelPWE::print(int verbosity)
    {
        yDebug("[TAXEL] %s", iCub::skinDynLib::Taxel::toString(verbosity).c_str());
        if (verbosity > 3)
        {
            yDebug("[PWE] %s",pwe->toString(verbosity).c_str());
        }
    }

    string TaxelPWE::toString(int verbosity)
    {
        stringstream res;
        res << "[TAXEL] " << iCub::skinDynLib::Taxel::toString(verbosity);

        if (verbosity)
        {
            res << "[PWE] " << pwe->toString(verbosity);
        }
        return res.str();
    }

    bool TaxelPWE::resetParzenWindowEstimator()
    {
        pwe->resetAllHist();
        return true;
    }
    
    bool TaxelPWE::computeResponse(double stress_modulation)
    {

        Resp = 0.0;
        if(Evnts.empty())
        {
            //printf("[TaxelPWE::computeResponse()] Taxel ID: %d, no events for this taxel - Resp=0 and exiting.\n",this->getID());
            return true;
        }
        else
        {
            //printf("[TaxelPWE::computeResponse()] Taxel ID: %u, there are %u events to process.\n",this->getID(),Evnts.size());
            double locResp = 0.0;
            double maxResp = 0.0;
            std::vector<double> In(2);
            for(vector<IncomingEvent4TaxelPWE>::iterator it = Evnts.begin(); it!=Evnts.end(); it++)
            {
                if (insideRFCheck(*it))
                {
                   In[0] = it->getNRM();
                   In[1] = it->getTTC();
                   locResp = pwe->computeResponse(In);
                   //printf("  event: %s \n",it->toString().c_str());
                   //printf("\t locResp = locResp  + locResp * min(1.0,Evnt.Threat + stress_modulation)\n");
                   //printf("\t    = %.2f  + %.2f * min(1.0,%.2f + %.2f)\n",locResp,locResp,it->Threat,stress_modulation);
                   locResp = locResp + (locResp * min(1.0,it->Threat + stress_modulation)); //with this amplification,
                   //may come out of the range (which used to be <0,255>, now <0,1> after 9.8.2016)
                   //- in fact up to double that range
                   //printf("\t locResp  = %.2f  \n",locResp);
                   if (locResp > maxResp)
                       maxResp = locResp;
                }
                else
                    yWarning("[TaxelPWE::computeResponse()] Taxel ID: %d, event outside RF - should not be happening in current implementation. Event in Taxel FoR\n %s \n",this->getID(), it->toString().c_str());
            }
            if (maxResp > 0.0)
            {
                //printf(" Setting taxel response to maxResp: %.2f\n",maxResp);
                Resp = maxResp;
            }
            //else
                //printf("\t maxResp was <=0 (%.2f) - Leaving taxel Resp 0.\n",maxResp);
            
            return true;
        }
     }

    Bottle TaxelPWE::TaxelPWEIntoBottle()
    {
        Bottle res;
        res.clear();
        res.addInt(ID);

        Bottle &dataPH = res.addList();
        matrixOfIntIntoBottle(pwe->getPosHist(),dataPH);

        Bottle &dataNH = res.addList();
        matrixOfIntIntoBottle(pwe->getNegHist(),dataNH);

        return res;
    }

    TaxelPWE::~TaxelPWE()
    {
        if (pwe!=NULL)
        {
            delete pwe;
        }
    }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE 1D                                     */
/****************************************************************/

    TaxelPWE1D::TaxelPWE1D() : TaxelPWE()
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n) : TaxelPWE(p,n)
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n,
                           const int &i) : TaxelPWE(p,n,i)
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const Taxel &_t)
    {
        *this = _t;
    }

    TaxelPWE1D::TaxelPWE1D(const TaxelPWE1D &_t)
    {
        *this = _t;
    }

    TaxelPWE1D & TaxelPWE1D::operator=(const Taxel &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        // if (pwe)
        // {
        //     delete pwe;
        // }
        pwe = new parzenWindowEstimator1D();

        return *this;
    }

    TaxelPWE1D & TaxelPWE1D::operator=(const TaxelPWE1D &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        Evnts = t.Evnts;

        if (pwe)
        {
            delete pwe;
        }

        parzenWindowEstimator1D* newpwe = dynamic_cast<parzenWindowEstimator1D*>(t.pwe);
        pwe = new parzenWindowEstimator1D(*(newpwe));

        return *this;
    }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE 2D                                     */
/****************************************************************/

    TaxelPWE2D::TaxelPWE2D() : TaxelPWE()
    {
        pwe = new parzenWindowEstimator2D();
    }

    TaxelPWE2D::TaxelPWE2D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n) : TaxelPWE(p,n)
    {
        pwe = new parzenWindowEstimator2D();
    }

    TaxelPWE2D::TaxelPWE2D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n,
                           const int &i) : TaxelPWE(p,n,i)
    {
        pwe = new parzenWindowEstimator2D();
    }


    TaxelPWE2D::TaxelPWE2D(const Taxel &_t)
    {
        *this = _t;
    }

    TaxelPWE2D::TaxelPWE2D(const TaxelPWE2D &_t)
    {
        *this = _t;
    }

    TaxelPWE2D & TaxelPWE2D::operator=(const Taxel &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        // if (pwe)
        // {
        //     delete pwe;
        // }
        pwe = new parzenWindowEstimator2D();

        return *this;
    }

    TaxelPWE2D & TaxelPWE2D::operator=(const TaxelPWE2D &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        Evnts = t.Evnts;

        if (pwe)
        {
            delete pwe;
        }

        parzenWindowEstimator2D* newpwe = dynamic_cast<parzenWindowEstimator2D*>(t.pwe);
        pwe = new parzenWindowEstimator2D(*(newpwe));

        return *this;
    }

// empty line to make gcc happy
