#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "interval.h"
#include "vec3.h"
#include "vertex_info.h"

vertex_info DUMMY_VERTEX(vec3(0, 0, 0));

class triangle : public hittable
{
private:
    point3 v0;
    point3 v1;
    point3 v2;
    shared_ptr<material> mat;

public:
    vertex_info &v0_info;
    vertex_info &v1_info;
    vertex_info &v2_info;
    vec3 face_normal;
    triangle(point3 v0, point3 v1, point3 v2, shared_ptr<material> material)
        : v0(v0), v1(v1), v2(v2), mat(material), v0_info(DUMMY_VERTEX), v1_info(DUMMY_VERTEX), v2_info(DUMMY_VERTEX), face_normal(vec3(0, 0, 0))
    {
        vec3 v01 = v1 - v0;
        vec3 v02 = v2 - v0;
        face_normal = unit_vector(cross(v01, v02));
    }

    triangle(point3 v0, point3 v1, point3 v2, vertex_info &vn0, vertex_info &vn1, vertex_info &vn2, shared_ptr<material> material)
        : v0(v0), v1(v1), v2(v2), mat(material), v0_info(vn0), v1_info(vn1), v2_info(vn2), face_normal(vec3(0, 0, 0))
    {
        vec3 v01 = v1 - v0;
        vec3 v02 = v2 - v0;
        face_normal = unit_vector(cross(v01, v02));
    }

    /**
     * Implementation of MT algorithm
    */
    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        vec3 v01 = v1 - v0;
        vec3 v02 = v2 - v0;
        // vec3 pvec = cross(r.direction(), v02);
        vec3 pvec = cross(r.direction(), v02);
        double determinant = dot(v01, pvec);
        if (fabs(determinant) < epsilon)
            return false; // did not hit triangle

        double inverse_determinant = 1 / determinant;

        vec3 tvec = r.origin() - v0;
        double u = dot(tvec, pvec) * inverse_determinant;
        if (u < 0 || u > 1)
            return false;

        vec3 qvec = cross(tvec, v01);
        double v = dot(r.direction(), qvec) * inverse_determinant;
        if (v < 0 || u + v > 1)
            return false;
        rec.t = dot(v02, qvec) * inverse_determinant;
        if (!ray_t.surrounds(rec.t))
            return false;
        rec.p = r.at(rec.t);

#if ENABLE_SMOOTH_SHADING
        //calculate vertex normals
        vec3 n0 = v0_info.sum_face_normals / v0_info.face_count;
        vec3 n1 = v1_info.sum_face_normals / v1_info.face_count;
        vec3 n2 = v2_info.sum_face_normals / v2_info.face_count;
        rec.normal = (1 - u - v) * n0 + (u * n1) + (v * n2);
#else
        rec.normal = face_normal;
#endif

        rec.mat = mat;
        rec.set_face_normal(r, rec.normal);
        return true;
    }
};
#endif