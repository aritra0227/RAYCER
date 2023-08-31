#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "interval.h"
#include "vec3.h"

class triangle : public hittable
{
private:
    point3 v0;
    point3 v1;
    point3 v2;
    shared_ptr<material> mat;

public:
    triangle(point3 v0, point3 v1, point3 v2, shared_ptr<material> material)
        : v0(v0), v1(v1), v2(v2), mat(material) {}

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
        double u = dot(tvec, pvec)*inverse_determinant;
        if(u < 0 || u > 1) return false;

        vec3 qvec = cross(tvec, v01);
        double v = dot(r.direction(), qvec)*inverse_determinant;
        if(v < 0 || u+v > 1) return false;
        rec.t = dot(v02, qvec)*inverse_determinant;
        if (!ray_t.surrounds(rec.t)) return false;
        rec.p = r.at(rec.t);
        rec.normal = unit_vector(cross(v01, v02));
        rec.mat = mat;
        rec.set_face_normal(r, rec.normal);
        return true;
    }
};
#endif