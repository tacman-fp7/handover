/**
 * Copyright (C) 2013 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
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
 * \author Alessandro Roncone
 *
 * Date: first release 30/10/2013
 *
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
**/

#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinPart.h>

#include <map>
#include <list>
#include <sstream>

#include "parzenWindowEstimator.h"

/**
 * Closes properly a given port
**/
void closePort(yarp::os::Contactable *_port);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as integers
**/
void matrixOfIntIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);

/**
* Converts an int to a string
**/
std::string int_to_string( const int a );

/**
* Computes the factorial using a recursive method
**/
unsigned int factorial(unsigned int n);

/**
* Struct that encloses all the information related to a stimulus/event (approaching object).
**/
struct IncomingEvent
{
    yarp::sig::Vector Pos;
    yarp::sig::Vector Vel;
    double Radius;          // average radius of the object
    std::string Src;       // the source of information the event is coming from
    double Threat; //negative valence that may be associated with an object; range <0,1>; 0 should be treated like a neutral object; 1 maximum threat

    double NRM; // distance of event from taxel with sign (meters) 
    double TTC; //time to contact (seconds)

    /**
    * Constructors
    **/
    IncomingEvent();
    IncomingEvent(const yarp::sig::Vector &p, const yarp::sig::Vector &v,
                  const double r, const std::string &s);
    IncomingEvent(const yarp::sig::Vector &p, const yarp::sig::Vector &v,
                  const double r, const double threat, const std::string &s);
    IncomingEvent(const yarp::os::Bottle &b);
    IncomingEvent(const IncomingEvent &e);

    /**
    *
    **/
    yarp::os::Bottle toBottle();

    /**
    *
    **/
    bool fromBottle(const yarp::os::Bottle &b);

    /**
    * Copy Operator
    **/
    IncomingEvent &operator=(const IncomingEvent &e);

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    std::string toString() const;
};

/**
* It has only a couple more stuff
**/
struct IncomingEvent4TaxelPWE : public IncomingEvent
{
    double NRM; // distance of event from taxel with sign (meters)
    double TTC; //time to contact (seconds)

    /**
    * Constructors
    **/
    IncomingEvent4TaxelPWE();
    IncomingEvent4TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &v,
                           const double r, const std::string &s);
    IncomingEvent4TaxelPWE(const IncomingEvent &e);
    IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e);

    /**
    * Copy Operators
    **/
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent &e);
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent4TaxelPWE &e);

    /**
    * Compute the NRM and TTC from Pos and Vel
    */
    void computeNRMTTC();

    /**
     * Return norm and TTC in a pwe-compliant way
    */
    std::vector<double> getNRMTTC();

    /**
     * Return norm (~ distance in meters with sign)
    */
    double getNRM();

    /**
     * Return time to contact (in seconds)
    */
    double getTTC();

    
    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    std::string toString() const;
};

/**
* Struct that encloses all the information related to the eyes.
* It's used for the projections.
**/
class eyeWrapper
{
public:
    std::string name;

    iCub::iKin::iCubEye *eye;

    double headVersion;
    yarp::sig::Matrix *Prj;

public:
    /**
    * Constructor
    **/
    eyeWrapper(std::string _name, double _hV, const yarp::os::ResourceFinder &_eyeCalibRF);

    /**
    * Copy Operator
    **/
    eyeWrapper &operator=(const eyeWrapper &ew);
};

#endif

// empty line to make gcc happy
