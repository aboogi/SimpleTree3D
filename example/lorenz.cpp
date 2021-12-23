/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

/****************************************************************************\
 **                                                            (C)1999 RUS   **
 **                                                                          **
 ** Description: Simple Geometry Generation Module                           **
 **              supports interaction from a COVER plugin                    **
 **              feedback style is later than COVISE 4.5.2                   **
 **                                                                          **
 ** Name:        cube                                                        **
 ** Category:    examples                                                    **
 **                                                                          **
 ** Author: D. Rainer                                                        **
 **                                                                          **
 ** History:                                                                 **
 ** September-99                                                             **
 **                                                                          **
 **                                                                          **
\****************************************************************************/

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#include "lorenz.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <api/coFeedback.h>
//#include <do/coDoPolygons.h>
#include <do/coDoPoints.h>
#include <do/coDoData.h>
#include <do/coDoSet.h>
#include <vector>

Lorenz::Lorenz(int argc, char *argv[]) // vvvv --- this info appears in the module setup window
    : coModule(argc, argv, "Simple Cube Generation Module")
{
    // output port
    // parameters:
    //   port name
    //   string to indicate connections, convention: name of the data type
    //   description
    p_ptsOut = addOutputPort("points", "Points", "Lorenz");
    p_fieldOut = addOutputPort("field", "Float", "A field");

    // input parameters
    // parameters:
    //   parameter name
    //   parameter type
    //   description
    p_start = addFloatVectorParam("center", "Center of the cube");
    p_start->setValue(3.051522, 1.582542, 15.62388);

    p_Sigma = addFloatSliderParam("sigma", "Sigma param");
    p_Sigma->setValue(1.0f, 20.0f, 10.0f);

    p_B = addFloatSliderParam("B", "B param");
    p_B->setValue(0.0f, 10.0f, 8.0f/3);

    p_R = addFloatSliderParam("R", "R param");
    p_R->setValue(10.0f, 100.0f, 15.0f);

    p_dt = addFloatParam("dt", "step");
    p_dt->setValue(0.0001f);

    p_Npoints = addInt32Param("Npoint", "Number of points");
    p_Npoints->setValue(10000);

    p_Anim = addBooleanParam("Animate", "Does animate?");
    p_Anim->setValue(false);

}

void Lorenz::postInst()
{
    p_start->show();
    p_Sigma->show();
}

int Lorenz::compute(const char *port)
{
    using namespace std;

    int nPoints = p_Npoints->getValue();
    float dt = p_dt->getValue();
    float sigma = p_Sigma->getValue();
    float b = p_B->getValue();
    float r = p_R->getValue();

    vector<float> x(nPoints), y(nPoints), z(nPoints), f(nPoints);

    f[0]=0;
    p_start->getValue(x[0], y[0], z[0]);
    for (int i=1; i<nPoints; i++)
    {
        f[i] = i;
        x[i] = x[i-1]+dt*sigma*(y[i-1]-x[i-1]);
        y[i] = y[i-1]+dt*(x[i-1]*(r-z[i-1])-y[i-1]);
        z[i] = z[i-1]+dt*(x[i-1]*y[i-1]-b*z[i-1]);
    }





    // get the data object name from the controller
    const char *ptsObjName = p_ptsOut->getObjName();
    const char *fieldObjName = p_fieldOut->getObjName();
    if (ptsObjName==nullptr || fieldObjName==nullptr)
    {
        fprintf(stderr, "covise failed\n");
        return FAIL;

    }

    if (p_Anim->getValue())
    {
        cout << "On animate" << endl;
        coDistributedObject **Pts = new coDistributedObject* [nPoints+1];
        coDistributedObject **Field = new coDistributedObject* [nPoints +1];
        for (int i=0; i<nPoints; i++)
        {
            char ptsName[2048];
            char fieldName[2048];
            sprintf(ptsName, "%s_%d", ptsObjName, i);
            sprintf(fieldName, "%s_%d", fieldObjName, i);

            Pts[i] = new coDoPoints(ptsName, i+2, x.data(), y.data(), z.data());
            Field[i] = new coDoFloat(fieldName, i+2, f.data());
        }
        Pts[nPoints-1] = nullptr;
        Field[nPoints-1] = nullptr;

        coDoSet *DoPts = new coDoSet(ptsObjName, Pts);
        coDoSet *DoField = new coDoSet(fieldObjName, Field);
        char attrStr[16];
        sprintf(attrStr, "0 %d", nPoints-1);
        DoPts-> addAttribute("TIMESTEP", attrStr);
        DoField->addAttribute("TIMESTEP", attrStr);
        p_ptsOut->setCurrentObject(DoPts);
        p_fieldOut->setCurrentObject(DoField);

        for (int i = 0; i < nPoints-1; ++i) {
            delete Pts[i];
            delete Field[i];
        }
        delete [] Pts;
        delete [] Field;
    }
    else
    {
        coDoPoints *ptsObj = new coDoPoints(ptsObjName, nPoints,  &x[0], y.data(), z.data());
        coDoFloat *fieldObj = new coDoFloat(fieldObjName, nPoints, f.data());

        p_ptsOut->setCurrentObject(ptsObj);
        p_fieldOut->setCurrentObject(fieldObj);
    }
    return SUCCESS;
}

Lorenz::~Lorenz()
{
}

void Lorenz::param(const char *name, bool /*inMapLoading*/)
{

    //    if (strcmp(name, p_Sigma->getName()) == 0)
    //    {
    //        sVal = p_Sigma->getValue();
    //    }
    //    else if (strcmp(name, p_start->getName()) == 0)
    //    {
    //        cx = p_start->getValue(0);
    //        cy = p_start->getValue(1);
    //        cz = p_start->getValue(2);
    //    }
}

MODULE_MAIN(Students, Lorenz)
