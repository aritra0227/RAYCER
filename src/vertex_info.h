#ifndef VERTEX_INFO_H
#define VERTEX_INFO_H
#include "vec3.h"


//to calculate vertex normals if not provided in .obj files
struct vertex_info
{
    vec3 vertex_pointer;
    vec3 sum_face_normals;
    int face_count;
    vertex_info(point3 v) : vertex_pointer(v), sum_face_normals(vec3(0, 0, 0)), face_count(0) {}
};



#endif