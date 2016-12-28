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

#ifndef __SKINPARTPWE_H__
#define __SKINPARTPWE_H__

#include "utils.h"
#include "taxelPWE.h"

class skinPartPWE : public iCub::skinDynLib::skinPart
{
  public:
    /**
    * Modality (either 1D or 2D)
    */
    std::string modality;

    /*
    * Constructor that assigns modality member
    **/
    skinPartPWE(const std::string &_modality);

    /**
    * Destructor
    **/
    ~skinPartPWE();

    /**
    * Copy Constructor
    * @param _spwe is the skinPartPWE to copy from
    **/
    skinPartPWE(const skinPartPWE &_spwe);

    /**
    * Copy Operator
    **/
    skinPartPWE &operator=(const skinPartPWE &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    std::string toString(int precision=0);
};

#endif

// empty line to make gcc happy
