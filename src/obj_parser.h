#ifndef OBJ_PARSER_H
#define OBJ_PARSER_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "vec3.h"

using namespace std;
// world.add(make_shared<mesh>(npolys, faceIndex, vertsIndex, P, material_center));

class Parser
{
    // private:

public:
    int num_faces;
    std::unique_ptr<unsigned int[]> face_index;
    std::unique_ptr<unsigned int[]> vertex_index;
    std::unique_ptr<vec3[]> vertices;
    Parser() {}

    /**
    * Used to parse .obj files
    */
    int parse_obj(string file_name)
    {
        ifstream inputFile(file_name);
        if (!inputFile)
        {
            cerr << "Error opening file!" << endl;
            return 1;
        }
        string line;
        std::vector<vec3> vertices_vec;
        std::vector<unsigned int> vertex_index_vec;
        std::vector<unsigned int> face_index_vec;
        double x, y, z;
        char skip;
        int curr_vertex_index, dummy_index, num_vertices;
        bool first_f_line = true;
        enum fformat_t
        {
            V,
            VTN,
            VN
        }; // V = Vertex, T = Texture, N = Normal
        fformat_t f_type;

        while (std::getline(inputFile, line))
        {
            //store vertices
            if (line[0] == 'v' && line[1] == ' ')
            {
                std::istringstream ss(line);
                ss >> skip >> x >> y >> z;
                vertices_vec.emplace_back(vec3(x, y, z));
            }

            // store faces
            // for now only get indices for vertices
            else if (line[0] == 'f')
            {
                std::istringstream ss(line);
                num_vertices = 0;
                ss >> skip;
                if (first_f_line)
                {
                    // num_vertices += 1;
                    if (line.find("//") != std::string::npos)
                        f_type = VN;

                    if (line.find("/") != std::string::npos)
                    {
                       f_type = VTN;
                    }
                    else
                        f_type = V;
                    first_f_line = false;
                }

                while (ss >> curr_vertex_index)
                {
                    num_vertices += 1;
                    vertex_index_vec.push_back(curr_vertex_index - 1);
                    switch (f_type)
                    {
                    case V:
                        break;
                    case VTN:
                        ss >> skip >> dummy_index >> skip >> dummy_index;
                        break;
                    case VN:
                        ss >> skip >> skip >> dummy_index;
                        break;
                    }
                }
                face_index_vec.push_back(num_vertices);
            }
        }
        //copy over vectors to arrays:
        size_t len_vertices = vertices_vec.size();
        vertices = std::unique_ptr<vec3[]>(new vec3[len_vertices]);
        for (size_t i = 0; i < len_vertices; ++i)
        {
            vertices[i] = vertices_vec[i];
        }
        size_t len_vertex_indices = vertex_index_vec.size();
        vertex_index = std::unique_ptr<unsigned int[]>(new unsigned int[len_vertex_indices]);
        for (size_t i = 0; i < len_vertex_indices; ++i)
        {
            vertex_index[i] = vertex_index_vec[i];
        }

        size_t len_face_indices = face_index_vec.size();
        face_index = std::unique_ptr<unsigned int[]>(new unsigned int[len_face_indices]);
        for (size_t i = 0; i < len_face_indices; ++i)
        {
            face_index[i] = face_index_vec[i];
        }
        num_faces = len_face_indices;

        inputFile.close(); // Close the file
        return 0;
    }
};

#endif