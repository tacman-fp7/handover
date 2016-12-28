/* 
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
*/

#ifndef __ICUBDBLTCHSLV_H__
#define __ICUBDBLTCHSLV_H__

#include "iKinFwdMod.h"

/**
 * \defgroup doubleTouchSlvLib doubleTouchSlvLib
 *  
 * @ingroup periPersonalSpace
 *  
 * Classes for solving the double touch, i.e. providing an inverse
 * kinematics solver able to achieve a successfull double touch.
 *  
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \author Alessandro Roncone - Ugo Pattacini 
 *  
 * Date: first release 30/06/2013
 *  
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */ 

/**
* @ingroup doubleTouchSlvLib
*
* A struct for defining the problem's variables (from the optimizer's standpoint). 
* Theroretically, a part from the type of task (R2L or L2R or whatever), any 
* double touch problem should be self-contained in such a class (apart from the
* chain, that is not a parameter :P)
*  
*/
struct doubleTouch_Variables
{    
    yarp::sig::Vector ee;
    yarp::sig::Vector joints;

    yarp::sig::Matrix H;
    yarp::sig::Matrix H_0;

    yarp::sig::Vector z_hat;
    yarp::sig::Vector x_hat;

    double dot;

    /**
    * Constructor. 
    * @param dim is the number of links of the chain to be dealt with. Anything
    *            else should be fitted by hand by the operator (and it's fine,
    *            because there's no need to hide the members of such a simple
    *            struct to the final user)
    */
    doubleTouch_Variables(int dim);

    /**
    * Prints the state of the variable
    */
    void print();

    /**
    * Copy Operator
    */
    doubleTouch_Variables &operator=(const doubleTouch_Variables &v);

    /**
    * Clone Function
    */
    void clone (const doubleTouch_Variables &v);
};

/**
* @ingroup doubleTouchSlvLib
*
* A struct for defining a double touch problem in its entirety. It's a wrapper for the underlying
* subproblems. Hence, any subproblem can be run at any time and they can be run in parallel without
* reconfiguring the solver over and over again.
*  
*/
struct doubleTouch_Problem
{
    iCubCustomLimb                 limb;
    iCub::iKin::iCubFinger         index;
    iCub::iKin::iKinLinIneqConstr *mLIC;
    iCub::iKin::iKinLinIneqConstr *sLIC;
    int                            nJoints;
    int                            nVars;
    doubleTouch_Variables          guess;

    std::string                    getType()    { return limb.getType(); }
    int                            getNVars()   { return nVars; }
    iCub::iKin::iKinLinIneqConstr* getMLIC()    { return mLIC; }
    iCub::iKin::iKinLinIneqConstr* getSLIC()    { return sLIC; }
    iKinChainMod*                  asChainMod() { return limb.asChainMod(); }
    iCubCustomLimb*                getLimb()    { return &limb; }

    /**
    * Constructor. 
    * @param _type is the type of subproblem that has to be instantiated. Right now, it can be
    *              'RtoL','LtoR','RHtoL','LHtoR'
    */
    doubleTouch_Problem(std::string _type, std::string _indextype);

    /**
     * Copy operator.
     */
    doubleTouch_Problem & operator=(const doubleTouch_Problem &sp);

    /**
     * Destructor.
     */
    ~doubleTouch_Problem();
};

/**
* @ingroup doubleTouchSlvLib
*
* A class for implementing a solver for doubletouch. It what is needed, and nothing more: the
* problem, a pointer to the subproblem, a way to detect the problem under investigation.
*  
*/
class doubleTouch_Solver
{
    public:
        doubleTouch_Problem *probl;

        /**
        * Constructor. 
        * @param type is the type of subproblem that has to be instantiated. Right now, it can be
        *             either 'RtoL','LtoR','RHtoL', or 'LHtoR'
        */
        doubleTouch_Solver(std::string _type);

        /**
        * Sets the initial parameters from which starting the optimization.
        */
        void setInitialGuess(const doubleTouch_Variables &g);

        /**
        * Solves the Inverse Kinematics task associated with the double touch.
        */
        bool solve(doubleTouch_Variables &solution);

        /**
         * Destructor.
         */
        ~doubleTouch_Solver();
};

#endif

// empty line to make gcc happy
