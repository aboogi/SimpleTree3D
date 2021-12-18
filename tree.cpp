#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#include "tree.h"
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "glm/geometric.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/quaternion.hpp"

#include "do/coDoLines.h"
#include "do/coDoPoints.h"
#include "do/coDoCoordinates.h"
#include "do/coDoBasisTree.h"
#include "do/coDoData.h"

Tree::Tree(int argc, char *argv[])
    : coModule(argc, argv, "Tree generated")
{
    //output ports
    //parameters:
    //   port name
    //   string to indicate connections, convention: name of the data type
    //   description

    //Port
    p_LinesOut = addOutputPort("lines", "Lines", "Tree");
    p_fieldOut = addOutputPort("field", "Float", "Scalar field");

    p_Length = addFloatSliderParam("Length", "Length of tree");
    p_Length->setValue(1.0f, 20, 1.0f);

    p_Scale = addFloatSliderParam("Scale", "Scale of decrease length on next level");
    p_Scale->setValue(0.0f, 1.0f, 0.5f);

    p_Deep = addIntSliderParam("Deep", "Levels of branch");
    p_Deep->setValue(0, 100, 2);

    p_Start = addFloatVectorParam("start", "Center of the cube");
    p_Start->setValue(0.0f, 0.0f, 0.0f);
}

void Tree::postInst()
{
    p_Start->show();
}

int Tree::compute(const char *port)
{
    using namespace std;

    float b_length = p_Length->getValue();
    float b_scale = p_Scale->getValue();
    int b_deep = p_Deep->getValue();

    int count_vertex = getVertexCount(b_deep);
    vector<float> x_coord(1), y_coord(1), z_coord(1), f(1);

    p_Start->getValue(x_coord[0], y_coord[0], z_coord[0]);
//    f[0] = b_length;

    x_coord.push_back(x_coord[0]);
    y_coord.push_back(y_coord[0]);
    z_coord.push_back(z_coord[0] + b_length);

    //    vector < vector<float>> *xyz_coord = new {x_coord, y_coord, z_coord};

    LinesData *lines_data = new LinesData();
    lines_data->xyz_coord = {x_coord, y_coord, z_coord};
    lines_data->num_lines = {0};
    lines_data->line_corners = {0, 1};
    lines_data->f = {0,0};

    if (b_deep != 0)
    {
        genTree(lines_data, b_length, b_deep, b_scale, 0, 1, -1);
//        lines_data->f.push_back((lines_data->f[(lines_data->f).size() - 1]));
//        lines_data->f.push_back(0.25);
    }

    const char *lines_ObjName = p_LinesOut->getObjName();
    const char *field_ObjName = p_fieldOut->getObjName();
    if (lines_ObjName==nullptr || field_ObjName==nullptr){
        cout << (stderr, "covise failed. Output data check")<<endl;
        return FAIL;
    }
    else // (lines_ObjName)
    {
        cout << "HEEEEEEEEEEEEEEEEEET" << endl;
        cout << count_vertex << " " << (lines_data->line_corners).size() << " " << (lines_data->line_corners).data() << endl;
        cout << (lines_data->num_lines).size() << " " << (lines_data->num_lines).data() << endl;
//        coDistributedObject** ln = new coDistributedObject* [(lines_data->num_lines).size()+1];
//        coDistributedObject** field = new coDistributedObject* [(lines_data->f).size()+1];



        coDoLines *line_obj = new coDoLines(lines_ObjName,
                                            count_vertex,
                                            (lines_data->xyz_coord)[0].data(), (lines_data->xyz_coord)[1].data(), (lines_data->xyz_coord)[2].data(),
                                            (lines_data->line_corners).size(),
                                            (lines_data->line_corners).data(),
                                            (lines_data->num_lines).size(),
                                            (lines_data->num_lines).data());

        coDoFloat *fieldObj = new coDoFloat(field_ObjName, (lines_data->f).size(), (lines_data->f).data());

        p_LinesOut->setCurrentObject(line_obj);
        p_fieldOut->setCurrentObject(fieldObj);

    }
//    else(lines_data->f).push_back(current_level);
//    {
//        cout << "LOG 3DTree: Smth wrongs with output lines data" << endl;
//        return FAIL;
//    }
    return SUCCESS;
}

void genTree(LinesData *lines_data, float b_length, int b_deep, float b_scale, int current_level, int last_index_pts, int p_prevPts)
{
    vector<float>::iterator it;
    vector<vector<float>> coords(3);

    float rot_x_1, rot_x_2, rot_x_3;
    float rot_y_1, rot_y_2, rot_y_3;
    float rot_z_1, rot_z_2, rot_z_3;

    float prev_x, p_prev_x;
    float prev_y, p_prev_y;
    float prev_z, p_prev_z;

    glm::vec3 p_prev, prev;

    current_level++;
    b_length *= b_scale;

    // All changes with coordinates
    prev_x = lines_data->xyz_coord[0][last_index_pts];
    prev_y = lines_data->xyz_coord[1][last_index_pts];
    prev_z = lines_data->xyz_coord[2][last_index_pts];
    prev = glm::vec3(prev_x, prev_y, prev_z);

    if (p_prevPts > -1)
    {
        p_prev_x = lines_data->xyz_coord[0][p_prevPts];
        p_prev_y = lines_data->xyz_coord[1][p_prevPts];
        p_prev_z = lines_data->xyz_coord[2][p_prevPts];
        p_prev = glm::vec3(p_prev_x, p_prev_y, p_prev_z);
    }
    else
    {
        p_prev = glm::vec3(0.0f);
    }

    if (current_level == 1)
    {
        float x = 0;
        float y = 0;
        float z = b_length;

        // First rotate around OX
        rot_x_1 = x;
        rot_y_1 = y * cos(glm::radians(90.0f)) + z * sin(glm::radians(90.0f));
        rot_z_1 = -y * sin(glm::radians(90.0f)) + z * cos(glm::radians(90.0f));

        // Second rotate around OZ
        rot_x_2 = rot_x_1 * cos(glm::radians(120.0f)) - rot_y_1 * sin(glm::radians(120.0f));
        rot_y_2 = -rot_x_1 * sin(glm::radians(120.0f)) + rot_y_1 * cos(glm::radians(120.0f));
        rot_z_2 = rot_z_1;

        // Third rotate around OZ
        rot_x_3 = rot_x_1 * cos(glm::radians(240.0f)) - rot_y_1 * sin(glm::radians(240.0f));
        rot_y_3 = -rot_x_1 * sin(glm::radians(240.0f)) + rot_y_1 * cos(glm::radians(240.0f));
        rot_z_3 = rot_z_1;
    }
    else
    {
        // Compute norm vector
        glm::vec3 norm_vec; // vector C = cross(A, B)
        glm::vec4 norm_vec4;
        glm::vec3 pr;

        // norm_vec = glm::cross(prev, p_prev);
        norm_vec = glm::normalize(glm::cross(prev, p_prev));
        norm_vec4 = glm::vec4(norm_vec.x, norm_vec.y, norm_vec.z, 0.0f);

        pr = p_prev - prev; // Previus vector from origin

        // Transform matrix
        glm::mat4 mod = glm::mat4(1.0f);
        glm::mat4 m_rotor = glm::rotate(mod, glm::radians(120.0f), pr);

        glm::vec4 point_1 = norm_vec4 * b_length;
        glm::vec4 point_2 = m_rotor * point_1;
        glm::vec4 point_3 = m_rotor * point_2;

        rot_x_1 = point_1.x;
        rot_y_1 = point_1.y;
        rot_z_1 = point_1.z;

        rot_x_2 = point_2.x;
        rot_y_2 = point_2.y;
        rot_z_2 = point_2.z;

        rot_x_3 = point_3.x;
        rot_y_3 = point_3.y;
        rot_z_3 = point_3.z;

        cout << "get point" << endl;
    }

    //////////////////////////////////////

    // pts 1 in origin SC
    coords[0].push_back(rot_x_1 + prev_x);
    coords[1].push_back(rot_y_1 + prev_y);
    coords[2].push_back(rot_z_1 + prev_z);

    // pts 2 in origin SC
    coords[0].push_back(rot_x_2 + prev_x);
    coords[1].push_back(rot_y_2 + prev_y);
    coords[2].push_back(rot_z_2 + prev_z);

    // pts 3 in origin SC
    coords[0].push_back(rot_x_3 + prev_x);
    coords[1].push_back(rot_y_3 + prev_y);
    coords[2].push_back(rot_z_3 + prev_z);

    //////////////////////////////////////

    for (int i = 0; i < 3; i++)
    {
        vector<float> sep_vector;
        vector<float> &axis = (lines_data->xyz_coord)[i];

        it = axis.begin() + axis.size();
        sep_vector = coords[i];

        axis.insert(it, sep_vector.begin(), sep_vector.end());
    }

    for (int i = 0; i < 3; i++)
    {
        int index_coord = (lines_data->xyz_coord[0]).size() - 3 + i;

        (lines_data->line_corners).push_back(last_index_pts);
        (lines_data->line_corners).push_back(index_coord);

        (lines_data->num_lines).push_back((int)(lines_data->line_corners).size() - 2);

        (lines_data->f).push_back(current_level);
//        (lines_data->f).push_back(b_length);
    }
//    lines_data->f.push_back(lines_data->f[(lines_data->f).size() - 1]);
//    (lines_data->f).push_back(current_level);

    ///////////////////////////////////////////////////////////////////

    if (current_level != b_deep)
    {
        int x_size = (lines_data->xyz_coord[0]).size();

        genTree(lines_data, b_length, b_deep, b_scale, current_level, x_size - 3, last_index_pts);
        genTree(lines_data, b_length, b_deep, b_scale, current_level, x_size - 2, last_index_pts);
        genTree(lines_data, b_length, b_deep, b_scale, current_level, x_size - 1, last_index_pts);
    }
}

int getVertexCount(int b_deep)
{
    int num_nodes = 1;
    for (int i = 0; i <= b_deep; i++)
    {
        int power = b_deep - i;
        num_nodes += pow(3, power);
    }
    return num_nodes;
}

float DegToRad(float deg)
{
    float rad;

    rad = (deg * M_PI) / 180.0;

    return rad;
}

void Tree::param(const char *name, bool /*inMapLoading*/)
{}

Tree::~Tree()
{
}

MODULE_MAIN(ABA, Tree);
