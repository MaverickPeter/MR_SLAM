/*
 * GridUtilHash.hpp
 *
 *  Created on: Sept 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, Robotics 104
 */

#pragma once

#include <iostream>
#include <unordered_map>
#include <string>
#include <functional>
#include <math.h>
// #include <tuple>
#include <utility>

using namespace std;

// typedef tuple<float, float> tup;

struct GridPoint{
    float x;
    float y;
    GridPoint(float x_, float y_):x(x_), y(y_){}
};

struct GridPointData{
    float var;
    float travers;
    float elevation;
    float intensity;
    int frontier;
    int r;
    int g;
    int b;
    GridPointData(float ele_, float var_, int r_, int g_, int b_, int frontier_, float intensity_, float travers_):elevation(ele_), var(var_),r(r_),g(g_),b(b_),frontier(frontier_),intensity(intensity_),travers(travers_){}
};

struct GridPointHashFunc{
    size_t operator()(const GridPoint & p) const{
        int x = *(int*)(&p.x);
        int y = *(int*)(&p.y);
        using std::size_t;
        using std::hash;
        return hash<int>()(x) ^ hash<int>()(y) << 1;
    }
};

struct GridPointEqual{
    bool operator()(const GridPoint & p, const GridPoint & t) const{
        return p.x == t.x && p.y == t.y;
    }
};

// struct GridPointEqual: public binary_function<tup, tup, bool>{
//     bool operator()(const tup& v0, const tup& v1) const{
//         return get<0>(v0) == get<0>(v1) && get<1>(v0) == get<1>(v1);
//     }
// };

// struct GridPointHash: public unary_function<tup, size_t>{
//     size_t operator()(const tup& k) const{
//         return (get<0>(k) + get<1>(k) - (int)(get<0>(k) + get<1>(k))) * 0.85 * 1000000;
//     }
// };

// struct GridPointHash{
//     size_t operator()(const GridPoint& pt) const{
//         float k = 0.75;
//         return ((hash<float>()(pt.x) + hash<float>()(pt.y))
//                     - (int)(hash<float>()(pt.x) + hash<float>()(pt.y)) * k * 100000);
//     }
// };

// namespace std{
//     template <>
//     struct hash<GridPoint>{
//     public:
//         size_t operator()(const GridPoint& p) const{
//             float k = 0.103149002;
//             return (hash<float>()(p.x) + hash<float>()(p.y)) * k % 1 * 100000;
//         }
//     };
// }