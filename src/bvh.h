#ifndef BVH_H
#define BVH_H

#include "acceleration.h"
#include "interval.h"
#include "material.h"
#include <queue>

#define MAX_DEPTH 16
/**
 * TLAS: Top Level Acceleration Structure
 * Should mimic hittable_list since that is what we will be replacing:
*/

class BVH : public accel
{
private:
    static const int num_plane_set_normals = 7;
    static const vec3 plane_set_normals[num_plane_set_normals];
    /**
    * bounding box enclosing an object
    */
    struct bbox
    {
        interval bounds[num_plane_set_normals];
        shared_ptr<hittable> bounded_object;
        bbox() {}
        bool hit(double *NdotOrig, double *NdotDir, double &tnear, double &tfar, size_t &pi)
        {
            for (size_t i = 0; i < num_plane_set_normals; ++i)
            {
                double tn = (bounds[i].min - NdotOrig[i]) / NdotDir[i];
                double tf = (bounds[i].max - NdotOrig[i]) / NdotDir[i];
                if (NdotDir[i] < 0)
                    std::swap(tn, tf);
                if (tn > tnear)
                    tnear = tn, pi = i;
                if (tf < tfar)
                    tfar = tf;
                if (tnear > tfar)
                    return false;
            }
            return true;
        }
        void extend_bounds(const bbox &object_box)
        {
            for (size_t i = 0; i < num_plane_set_normals; ++i)
            {
                if (object_box.bounds[i].min < bounds[i].min)
                    bounds[i].min = object_box.bounds[i].min;
                if (object_box.bounds[i].max > bounds[i].max)
                    bounds[i].max = object_box.bounds[i].max;
            }
        }
    };

    bbox *objects_bounds;

    struct octnode
    {
        octnode *children[8] = {nullptr};
        std::vector<const bbox *> data;
        bbox box;
        bool is_leaf = true;
        unsigned int depth;
    };

    struct octree
    {
        octnode *root = nullptr;
        vec3 octree_bounds[2];
        octree(const bbox &boxes) : root(NULL)
        {
            double x_diff = boxes.bounds[0].max - boxes.bounds[0].min;
            double y_diff = boxes.bounds[1].max - boxes.bounds[1].min;
            double z_diff = boxes.bounds[2].max - boxes.bounds[2].min;
            double max_diff = std::max(x_diff, std::max(y_diff, z_diff));
            vec3 max_diff_vec = vec3(max_diff, max_diff, max_diff);
            vec3 centroid(boxes.bounds[0].max + boxes.bounds[0].min,
                          boxes.bounds[1].max + boxes.bounds[1].min,
                          boxes.bounds[2].max + boxes.bounds[2].min);
            octree_bounds[0] = (centroid - max_diff_vec) * 0.5f;
            octree_bounds[1] = (centroid + max_diff_vec) * 0.5f;
            root = new octnode;
        }
        ~octree() { delete_all_nodes(root); }
        void insert(const bbox *box)
        {
            insert(root, box, octree_bounds, 0);
        }
        void build()
        {
            build(root, octree_bounds);
        }
        //structure to optimize ending ending traversal if closest object intersected in found
        struct QE
        {
            const octnode *node;
            double t;
            QE(const octnode *n, double key) : node(n), t(key) {}
            friend bool operator<(const QE &a, const QE &b) { return a.t > b.t; }
        };

    private:
        void insert(octnode *node, const bbox *box, vec3 (&bounds)[2], int depth)
        {
            if (node->is_leaf)
            {
                if (node->data.size() == 0 || depth == MAX_DEPTH)
                {
                    //empty node, so push object
                    node->data.push_back(box);
                }
                else
                {
                    //non-empty, hence split into 8 children
                    node->is_leaf = false;
                    while (node->data.size())
                    {
                        insert(node, node->data.back(), bounds, depth);
                        node->data.pop_back();
                    }
                    insert(node, box, bounds, depth);
                }
            }
            else
            {
                // std::clog << "inside insert" << std::endl;
                vec3 centroid_box = (vec3(box->bounds[0].min, box->bounds[1].min, box->bounds[2].min) + vec3(box->bounds[0].max, box->bounds[1].max, box->bounds[2].max)) * 0.5;
                vec3 centroid_node = (bounds[0] + bounds[1]) * 0.5f;
                size_t index = 0;
                if (centroid_box.x() > centroid_node.x())
                    index += 4;
                if (centroid_box.y() > centroid_node.y())
                    index += 2;
                if (centroid_box.z() > centroid_node.z())
                    index += 1;
                vec3 child_bounds[2];
                vec3 centroid_bounds = (bounds[0] + bounds[1]) * 0.5;
                compute_child_bounds(index, centroid_bounds, bounds, child_bounds);
                if (node->children[index] == NULL)
                {
                    node->children[index] = new octnode;
                    node->children[index]->depth = depth;
                }
                insert(node->children[index], box, child_bounds, depth + 1);
            }
        }
        void compute_child_bounds(size_t index, vec3 &centroid_bounds, vec3 (&bounds)[2], vec3 (&child_bounds)[2])
        {
            double min_x = (index & 4) ? centroid_bounds.x() : bounds[0].x();
            double max_x = (index & 4) ? bounds[1].x() : centroid_bounds.x();
            double min_y = (index & 2) ? centroid_bounds.y() : bounds[0].y();
            double max_y = (index & 2) ? bounds[1].y() : centroid_bounds.y();
            double min_z = (index & 1) ? centroid_bounds.z() : bounds[0].z();
            double max_z = (index & 1) ? bounds[1].z() : centroid_bounds.z();
            child_bounds[0] = vec3(min_x, min_y, min_z);
            child_bounds[1] = vec3(max_x, max_y, max_z);
        }
        void build(octnode *node, vec3 (&bounds)[2])
        {
            if (node->is_leaf)
            {
                for (size_t i = 0; i < node->data.size(); ++i)
                {
                    node->box.extend_bounds(*node->data[i]);
                }
            }
            else
            {
                for (size_t i = 0; i < 8; ++i)
                {
                    if (node->children[i])
                    {
                        vec3 child_bounds[2]; //min,max
                        vec3 centroid_bounds = (bounds[0] + bounds[1]) * 0.5;
                        compute_child_bounds(i, centroid_bounds, bounds, child_bounds);
                        build(node->children[i], child_bounds);
                        node->box.extend_bounds(node->children[i]->box);
                    }
                }
            }
        }

        void delete_all_nodes(octnode *&node)
        {
            for (octnode *child : node->children)
            {
                if (child)
                {
                    delete_all_nodes(child);
                }
            }
            delete node;
        }
    };

    octree *tree = nullptr;

public:
    BVH() {}
    ~BVH() { delete tree; }
    void add(shared_ptr<hittable> object)
    {
        objects.push_back(object);
    }

    // must be called after adding all objects
    void set_up_bvh()
    {
        bbox scene_box;
        objects_bounds = new bbox[objects.size()];
        //calculate bounds for each object
        for (size_t i = 0; i < objects.size(); ++i)
        {
            for (size_t j = 0; j < num_plane_set_normals; ++j)
            {
                objects[i]->compute_bounds(plane_set_normals[j], objects_bounds[i].bounds[j].min, objects_bounds[i].bounds[j].max);
            }
            objects_bounds[i].bounded_object = objects[i];
            scene_box.extend_bounds(objects_bounds[i]);
        }
        tree = new octree(scene_box);
        for (size_t i = 0; i < objects.size(); ++i)
        {
            tree->insert(objects_bounds + i);
        }
        tree->build();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        //plane intersection equation: f(d) = (d - N.O)/(N.RD), ;;; . is for dot product
        //tnearest = f(dnearest), tfarthest = f(dfarthest);;; N = Normal, RD = ray direction
        //precompute N.O and N.RD
        bool hit_any_objects = false;
        double NdotOrig[num_plane_set_normals];
        double NdotDir[num_plane_set_normals];
        for (size_t i = 0; i < num_plane_set_normals; ++i)
        {
            NdotOrig[i] = dot(plane_set_normals[i], r.origin());
            NdotDir[i] = dot(plane_set_normals[i], r.direction());
        }
        // now just iterate for now to find the closest bbox:

#if DISABLE_SPACE_PARTITION
        double tclosest_object_so_far = ray_t.max;
        hit_record temp_rec;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            double tnear = ray_t.min, tfar = tclosest_object_so_far;
            size_t pi;
            if (objects_bounds[i].hit(NdotOrig, NdotDir, tnear, tfar, pi))
            {
                if (tnear < tclosest_object_so_far) //cloest bounding box hit
                {
                    if (objects[i]->hit(r, ray_t, temp_rec))
                    {
                        hit_any_objects = true;
                        tclosest_object_so_far = temp_rec.t;
                        rec = temp_rec;
                    }
                }
            }
        }

#else
        //add octree logic:
        size_t pi = 0;
        double tnear = 0, tfar = ray_t.max;
        if (!tree->root->box.hit(NdotOrig, NdotDir, tnear, tfar, pi) || tfar < 0 || tnear > ray_t.max)
        {
            // no intersection with the collection of objects/scene
            return false;
        }
        double t_min = tfar;
        std::priority_queue<BVH::octree::QE> que;
        que.push(BVH::octree::QE(tree->root, 0));
        //now basically bfs::
        while (!que.empty() && que.top().t < t_min)
        {
            const octnode *node = que.top().node;
            que.pop();
            if (node->is_leaf)
            {
                for (size_t i = 0; i < node->data.size(); ++i)
                {
                    hit_record temp_record;
                    if (node->data[i]->bounded_object->hit(r, ray_t, temp_record))
                    {
                        if (temp_record.t < t_min)
                        {
                            t_min = temp_record.t;
                            hit_any_objects = true;
                            rec = temp_record;
                        }
                    }
                }
            }
            else
            {
                for (size_t i = 0; i < 8; ++i)
                {
                    if (node->children[i] != NULL)
                    {
                        double tnear_child = 0, tfar_child = tfar;
                        if (node->children[i]->box.hit(NdotOrig, NdotDir, tnear_child, tfar_child, pi))
                        {
                            double t = (tnear_child < 0 && tfar_child >= 0) ? tfar_child : tnear_child;
                            que.push(BVH::octree::QE(node->children[i], t));
                        }
                    }
                }
            }
        }
#endif
        return hit_any_objects;
    }
};
const vec3 BVH::plane_set_normals[BVH::num_plane_set_normals] = {
    vec3(1, 0, 0),
    vec3(0, 1, 0),
    vec3(0, 0, 1),
    vec3((sqrtf(3) / 3.f), (sqrtf(3) / 3.f), (sqrtf(3) / 3.f)),
    vec3(-sqrtf(3) / 3.f, sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    vec3(-sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    vec3(sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f)};

#endif