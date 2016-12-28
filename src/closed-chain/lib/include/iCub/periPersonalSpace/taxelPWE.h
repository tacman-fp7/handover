/**
 * Copyright (C) 2013 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone, Matej Hoffmann
 * email:  alessandro.roncone@yale.edu, matej.hoffmann@iit.it
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
**/

/**
 * \defgroup utilsLib utilsLib
 *
 * @ingroup periPersonalSpace
 *
 * Utilities used throughout the modules and libraries.
 *
 * \author Alessandro Roncone, Matej Hoffmann
 *
 * Date: first release 30/10/2013
 *
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
**/

#ifndef __TAXELPWE_H__
#define __TAXELPWE_H__

#include "utils.h"

class TaxelPWE : public iCub::skinDynLib::Taxel
{
  public:
    double    Resp;  // Taxels' activation level <0,1>
    double RFangle;  // Angle of the receptive field [rad] - from the taxel normal
                     // The effective angle (also called aperture of a spherical sector) is thus double that 
    double sphericalSectorShiftOffset;  //offset by which the RF (spherical sector) will be shifted down along the z-axis (taxel normal)
    //we don't want to start with the apex at the taxel (where it would have 0 volume)
    //but we want to truncate it such that it starts at the height with a specified radius;  
                     
    std::vector<IncomingEvent4TaxelPWE> Evnts;    // Stimuli/events nearing the taxel in the taxel's FoR
    parzenWindowEstimator *pwe;     //

    /**
    * Constructors
    **/
    TaxelPWE();
    TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &n);
    TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &n, const int &i);

    /*
    * Destructor
    **/
    virtual ~TaxelPWE();

    /**
    * init function
    **/
    void init() { iCub::skinDynLib::Taxel::init(); };

    /**
    * Add or remove a sample from the pwe's histogram
    **/
    bool addSample(IncomingEvent4TaxelPWE ie);
    bool removeSample(IncomingEvent4TaxelPWE ie);

    /**
    * Check if the input sample is inside the Receptive field (a shifted spherical sector)
    **/
    bool insideRFCheck(const IncomingEvent4TaxelPWE ie);

    /**
    * Print Method
    **/
    virtual void print(int verbosity=0);

    /**
    * toString Method
    **/
    std::string toString(int verbosity=0);
    
    /**
    * Resets the parzen window estimator
    **/
    bool resetParzenWindowEstimator();
    
    /**
    * Computes the response of the taxel.
    * The computed response is stored inside the taxel Resp field.
    **/
    bool computeResponse(double stress_modulation);

    /**
    * Convert the taxel into a bottle in order to be saved on file
    **/
    virtual yarp::os::Bottle TaxelPWEIntoBottle();
};

class TaxelPWE1D : public TaxelPWE
{
  public:

    /**
    * Default Constructor
    **/
    TaxelPWE1D();

    /**
     * Constructor with position and normal vectors
     * @param _p is the position of the taxel
     * @param _n is the normal vector of the taxel
     */
    TaxelPWE1D(const yarp::sig::Vector &p,
               const yarp::sig::Vector &n);

    /**
     * Constructor with position, normal and ID
     * @param _p is the position of the taxel
     * @param _n is the normal vector of the taxel
     * @param _i is the ID of the taxel
     */
    TaxelPWE1D(const yarp::sig::Vector &p,
               const yarp::sig::Vector &n,
               const int &i);

    /**
    * Copy Constructor
    * @param _t is the Taxel to copy from
    **/
    TaxelPWE1D(const Taxel &_t);

    /**
    * Copy Constructor
    * @param _t is the Taxel to copy from
    **/
    TaxelPWE1D(const TaxelPWE1D &_t);

    /*
    * Copy Operator
    **/
    TaxelPWE1D &operator=(const TaxelPWE1D &t);

    /*
    * Copy Operator with the base class (i.e. Taxel)
    **/
    TaxelPWE1D &operator=(const Taxel &t);

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };

    /**
    * Print Method
    **/
    void print(int verbosity=0) { TaxelPWE::print(verbosity); };
};

class TaxelPWE2D : public TaxelPWE
{
  public:

    /**
    * Default Constructor
    **/
    TaxelPWE2D();

    /**
     * Constructor with position and normal vectors
     * @param _p is the position of the taxel
     * @param _n is the normal vector of the taxel
     */
    TaxelPWE2D(const yarp::sig::Vector &p,
               const yarp::sig::Vector &n);

    /**
     * Constructor with position, normal and ID
     * @param _p is the position of the taxel
     * @param _n is the normal vector of the taxel
     * @param _i is the ID of the taxel
     */
    TaxelPWE2D(const yarp::sig::Vector &p,
               const yarp::sig::Vector &n,
               const int &i);

    /**
    * Copy Constructor
    * @param _t is the Taxel to copy from
    **/
    TaxelPWE2D(const Taxel &_t);

    /**
    * Copy Constructor
    * @param _t is the Taxel to copy from
    **/
    TaxelPWE2D(const TaxelPWE2D &_t);

    /*
    * Copy Operator
    **/
    TaxelPWE2D &operator=(const TaxelPWE2D &t);

    /*
    * Copy Operator with the base class (i.e. Taxel)
    **/
    TaxelPWE2D &operator=(const Taxel &t);

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };

    /**
    * Print Method
    **/
    void print(int verbosity=0) { TaxelPWE::print(verbosity); };
};

#endif

// empty line to make gcc happy
