#ifndef MESH_H
#define MESH_H
#include "hittable.h"
#include "vec3.h"
#include "material.h"
#include "triangle.h"
using std::unique_ptr;

class mesh : public hittable
{
private:
    unsigned int num_triangles;
    shared_ptr<material> mat;
    std::vector<shared_ptr<triangle>> triangles;

public:
    mesh(const unsigned int num_faces, const unsigned int face_index[],
         const unsigned int vertex_index[],
         const vec3 vertices[],
         shared_ptr<material> material) : num_triangles(0), mat(material)
    {
        unsigned int k = 0, max_vertex_index = 0;
        for (unsigned int i = 0; i < num_faces; ++i)
        {
            num_triangles += face_index[i] - 2;
            for (unsigned int j = 0; j < face_index[i]; ++j)
            {
                if (vertex_index[k + j] > max_vertex_index)
                    max_vertex_index = vertex_index[k + j];
            }
            k += face_index[i];
        }
        max_vertex_index += 1;

        unique_ptr<vec3[]> triangle_vertices = unique_ptr<vec3[]>(new vec3[max_vertex_index]);
        for (unsigned int i = 0; i < max_vertex_index; ++i)
            triangle_vertices[i] = vertices[i];

        unique_ptr<unsigned int[]> triangle_vertex_index = unique_ptr<unsigned int[]>(new unsigned int[num_triangles * 3]);
        unsigned int curr_index = 0;
        for (unsigned int i = 0, k = 0; i < num_faces; ++i)
        {
            for (unsigned int j = 0; j < face_index[i] - 2; ++j)
            {
                triangle_vertex_index[curr_index] = vertex_index[k];
                triangle_vertex_index[curr_index + 1] = vertex_index[k + j + 1];
                triangle_vertex_index[curr_index + 2] = vertex_index[k + j + 2];
                curr_index += 3;
            }
            k += face_index[i];
        }
        //now store as triangle objects
        for (unsigned int i = 0; i < max_vertex_index; ++i)
        {
            triangles.push_back(std::make_shared<triangle>(triangle_vertices[triangle_vertex_index[i]],
                                                           triangle_vertices[triangle_vertex_index[i + 1]],
                                                           triangle_vertices[triangle_vertex_index[i + 2]], mat));
        }
    }

    /**
     * Note: Same logic as hittable_list
    */
    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        hit_record temp_rec;
        bool hit_anything = false;
        double closest_so_far = ray_t.max;

        for (const auto &triangle : triangles)
        {
            if (triangle->hit(r, interval(ray_t.min, closest_so_far), temp_rec))
            {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_anything;
    }
};

#endif