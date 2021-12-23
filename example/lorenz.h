/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _LORENZ_H
#define _LORENZ_H
/****************************************************************************\
 **                                                            (C)1999 RUS   **
 **                                                                          **
 ** Description: Simple Geometry Generation Module                           **
 **              supports interaction from a COVER plugin                    **
 **              feedback style is later than COVISE 4.5.2                   **
 **                                                                          **
 ** Name:        lorenz                                                        **
 ** Category:    students                                                    **
 **                                                                          **
 ** Author: D. Rainer		                                                **
 **                                                                          **
 ** History:  								                                **
 ** September-99  					       		                            **
 **                                                                          **
 **                                                                          **
\****************************************************************************/

#include <api/coModule.h>
using namespace covise;

class Lorenz : public coModule
{

private:
    float cx, cy, cz; // coodinates of the cubes p_center
    float sMin, sMax, sVal; // edge length of the cube

    //  member functions
    virtual int compute(const char *port);
    virtual void param(const char *name, bool inMapLoading);
    virtual void postInst();

    //  Ports
    coOutputPort *p_ptsOut;
    coFloatVectorParam *p_start;

    coFloatSliderParam *p_Sigma;
    coFloatSliderParam *p_B;
    coFloatSliderParam *p_R;

    coFloatParam *p_dt;
    coIntScalarParam *p_Npoints;

    coOutputPort *p_fieldOut;

    coBooleanParam *p_Anim;

public:
    Lorenz(int argc, char *argv[]);
    virtual ~Lorenz();
};
#endif
