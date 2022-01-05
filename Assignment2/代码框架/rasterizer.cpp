// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <iostream>
#include <algorithm>
#include <numeric>
#include <limits>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    std::vector<Eigen::Vector3f> polygonVectors;
    for (int i = 0; i < 3; ++i) {
        if (i == 2) {
            polygonVectors.push_back(_v[0] - _v[i]);
        } else {
            polygonVectors.push_back(_v[i + 1] - _v[i]);
        }
    }

    for (int i = 0; i < 3; ++i) {
        auto pointVector = Eigen::Vector3f { x, y, 0 } - _v[i];

        if (pointVector.cross(polygonVectors[i]).z() > 0) {
            return false;
        }
    }

    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    float left = std::numeric_limits<float>::max();
    float top = std::numeric_limits<float>::min();
    float right = std::numeric_limits<float>::min();
    float bottom = std::numeric_limits<float>::max();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    for (const auto &item : v) {
        if (item.x() < left) {
            left = item.x();
        }
        if (item.x() > right) {
            right = item.x();
        }
        if (item.y() > top) {
            top = item.y();
        }
        if (item.y() < bottom) {
            bottom = item.y();
        }
    }

    // If so, use the following code to get the interpolated z value.


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    for (int x = left; x < right; ++x) {
        for (int y = bottom; y < top; ++y) {
            super_sampling(t, x, y);
        }
    }
}

void rst::rasterizer::super_sampling(const Triangle &t, int x, int y) {
    double positive = 0;
    double stride = 1.0 / sampling_ratio;
    double sub_x = x - stride / sampling_ratio;
    double sub_y = y - stride / sampling_ratio;

    auto v = t.toVector4();

    for (int i = 0; i < sampling_ratio; ++i) {
        for (int j = 0; j < sampling_ratio; ++j) {
            double sampling_x = sub_x + i * stride;
            double sampling_y = sub_y + j * stride;

            if (insideTriangle(sampling_x, sampling_y, t.v)) {
                auto[alpha, beta, gamma] = computeBarycentric2D(sampling_x, sampling_y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                set_pixel(Eigen::Vector3f { x, y, z_interpolated }, t.getColor(), i, j);
            }
        }
    }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::vector<float>(sampling_ratio * sampling_ratio, std::numeric_limits<float>::infinity()));
        std::fill(color_buf.begin(), color_buf.end(), std::vector<Eigen::Vector3f>(sampling_ratio * sampling_ratio, Eigen::Vector3f {0, 0, 0}));
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    color_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color, const int sub_index_x, const int sub_index_y)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();

    if (depth_buf[ind][sub_index_x + sub_index_y * sampling_ratio] > point.z()) {
        depth_buf[ind][sub_index_x + sub_index_y * sampling_ratio] = point.z();
        color_buf[ind][sub_index_x + sub_index_y * sampling_ratio] = color / (sampling_ratio * sampling_ratio);
    }

    frame_buf[ind] = std::accumulate(
            color_buf[ind].begin(),
            color_buf[ind].end(),
            Eigen::Vector3f { 0, 0, 0 },
            [](Eigen::Vector3f& acc, Eigen::Vector3f& cur) {
                return acc + cur;
            });
}

// clang-format on