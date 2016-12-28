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
 * \defgroup parzenWindowEstimatorLib parzenWindowEstimatorLib
 *  
 * @ingroup periPersonalSpace
 *  
 * Discrete representation of peripersonal space + interpolation (smoothing) using Parzen windows.
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

#ifndef __PARZENWINDOWESTIMATOR_H__
#define __PARZENWINDOWESTIMATOR_H__

#include <yarp/os/Log.h>
#include <yarp/sig/Matrix.h>

#include <vector>
#include <sstream>
#include <cmath>

// These are basically some semi-useless constants. I can remove them but
// I've preserved them in order for me to ease the process of future
// improvements to the library
#define X_DIM 0
#define Y_DIM 1
#define START 0
#define END 1

/**
* Base class from which the pwe1D and the pwe2D will inherit
**/
class parzenWindowEstimator
{
  protected:
    int dim;                                // the dimension of the pwe (either 1[D] or 2[D])
    yarp::sig::Matrix ext;                  // the extension of the Receptive field (dim*2)

    std::vector<int>    binsNum;            // the number of partitions of the input space (x and y dimensions)
    std::vector<double> binWidth;           // the extension of the single sampling unit (x and y dimensions)

    std::vector<int>    firstPosBin;        // the first bin for which we have positive values (x and y dimensions)
    std::vector<double> firstPosBinShift;   // the shift from zero to the start value of the firstPosBin (x and y dimensions)

    std::vector<double> sigm;

    yarp::sig::Matrix posHist; // Histogram for the parzening - positive examples 
    yarp::sig::Matrix negHist; // Histogram for the parzening - negative examples 

  public:
    parzenWindowEstimator() {};
    parzenWindowEstimator(const yarp::sig::Matrix _ext, const std::vector<int> _binsNum);

    /**
    * Copy Constructor
    * @param _pwe is the parzenWindowEstimator to copy from
    **/
    parzenWindowEstimator(const parzenWindowEstimator &_pwe);

    /**
    * Copy Operator
    * @param _pwe is the parzenWindowEstimator to copy from
    **/
    parzenWindowEstimator &operator=(const parzenWindowEstimator &_pwe);

    /**
    * Resize the estimator to a given extension and number of samples
    * The histogram changes accordingly (and it's cleared as well).
    **/
    bool resize(const yarp::sig::Matrix _ext, std::vector<int> _binsNum);

    /**
    * Check an input is inside the receptive fields, and if so assigns
    * the proper histogram indexes that belong to that input vector.
    **/
    bool getIndexes(const std::vector<double> x, std::vector<int> &b);
    
    /**
    * Add or remove a sample from the histogram
    **/
    bool addSample(const std::vector<double> x);
    bool removeSample(const std::vector<double> x);

    /**
    * Get the value of the receptive field at a certain x
    **/
    virtual double getF_X(const std::vector<double> x) = 0;
    
    /**
    * Get the value of the receptive field at a certain x.
    * It differs from the previous because it's scaled
    **/
    virtual double getF_X_scaled(const std::vector<double> x) = 0;

    /**
    * Compute the response for a specific input sample
    **/
    double computeResponse(const std::vector<double> x);

    /**
    * Print Function
    **/
    virtual void print();

    /**
    * toString Method
    * @param verbosity is the verbosity level
    **/
    virtual std::string toString(int verbosity=0);

    /**
    * Self-explaining functions
    **/
    std::vector<int>    getHistSize()           { return binsNum; };
    std::vector<double> getBinWidth()           { return binWidth; };
    
    virtual yarp::sig::Matrix getExt()          { return ext; };

    int    getPosHist(int i, int j=0)           { return int(posHist(i,j)); };
    yarp::sig::Matrix getPosHist()              { return posHist; };
    int    getNegHist(int i, int j=0)           { return int(negHist(i,j)); };    
    yarp::sig::Matrix getNegHist()              { return negHist; };

    double getHist(int i, int j=0);
    yarp::sig::Matrix getHist();

    void setPosHist(int val, int i, int j=0)    { posHist(i,j) = val; };
    void setNegHist(int val, int i, int j=0)    { negHist(i,j) = val; };
    void setPosHist(const yarp::sig::Matrix &v) { posHist = v; };
    void setNegHist(const yarp::sig::Matrix &v) { negHist = v; };

    void resetAllHist()                         { posHist.zero(); negHist.zero(); };
};

/**
* class for defining a 1-D parzen window 
**/
class parzenWindowEstimator1D : public parzenWindowEstimator
{
  public:
    /**
    * Constructors
    **/
    parzenWindowEstimator1D();
    parzenWindowEstimator1D(const yarp::sig::Matrix _ext, const std::vector<int> _binsNum);

    /**
    * Copy Constructor
    * @param _pwe is the parzenWindowEstimator to copy from
    **/
    parzenWindowEstimator1D(const parzenWindowEstimator1D &_pwe);
        
    /**
    * Get the value of the receptive field at a certain x
    **/
    double getF_X(const std::vector<double> x);
    
    /**
    * Get the value of the receptive field at a certain x.
    * It differs from the previous because it's scaled
    * (i.e. its max is set to 255, the other values accordingly)
    **/
    double getF_X_scaled(const std::vector<double> x);

    /**
    * Print Function
    **/
    void print() { parzenWindowEstimator::print(); };
};

/**
* class for defining a 2-D parzen window with a custom range (even negative)
**/
class parzenWindowEstimator2D : public parzenWindowEstimator
{
  public:
    /**
    * Constructors
    **/
    parzenWindowEstimator2D();
    parzenWindowEstimator2D(const yarp::sig::Matrix _ext, const std::vector<int> _binsNum);

    /**
    * Get the value of the receptive field at a certain x
    **/
    double getF_X(const std::vector<double> x);
    
    /**
    * Get the value of the receptive field at a certain x.
    * It differs from the previous because it's scaled
    **/
    double getF_X_scaled(const std::vector<double> x);

    /**
    * Print Function
    **/
    void print() { parzenWindowEstimator::print(); };
};

#endif

// empty line to make gcc happy
