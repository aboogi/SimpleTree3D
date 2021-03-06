#ifndef _TREE_H
#define _TREE_H

#include "api/coModule.h"

using namespace covise;

class Tree : public coModule
{
private:
    float cx, cy, cz;

    //  member functions
    virtual int compute(const char *port);
    virtual void param(const char *name, bool inMapLoading);
    virtual void postInst();

    // Ports
    coOutputPort *p_fieldOut;
    coOutputPort *p_LinesOut;

    // Params
    coFloatVectorParam *p_Start;
    coFloatSliderParam *p_Scale;
    coFloatSliderParam *p_Length;

    coIntSliderParam *p_Deep;

public:
    Tree(int argc, char *argv[]);
    virtual ~Tree() = default;
};

float DegToRad(float deg);

struct LinesData
{
    vector <int> num_lines;
    vector <int> line_corners;
    vector <vector<float>> xyz_coord;
    vector <float> f;
};

void genTree(LinesData* lines_data, float b_length, int b_deep, float b_scale,
                int current_level, int last_index_pts, int p_prevPts);
int getVertexCount(int b_deep);

#endif // TREE_H

