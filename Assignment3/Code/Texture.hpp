//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
//        cv::resize(image_data, image_data, cv::Size(), 0.25, 0.25);

        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int floor_u_img = u_img;
        int floor_v_img = v_img;

        int ceil_u_img = ceil(u_img);
        int ceil_v_img = ceil(v_img);

        cv::Vec<float, 3> color00 = image_data.at<cv::Vec3b>(floor_v_img, floor_u_img);
        cv::Vec<float, 3> color01 = image_data.at<cv::Vec3b>(floor_v_img, ceil_u_img);
        cv::Vec<float, 3> color10 = image_data.at<cv::Vec3b>(ceil_v_img, floor_u_img);
        cv::Vec<float, 3> color11 = image_data.at<cv::Vec3b>(ceil_v_img, ceil_u_img);

        auto t = interpolate1d(color01, color11, v_img - floor_v_img);
        auto b = interpolate1d(color00, color10, v_img - floor_v_img);
        auto color = interpolate1d(b, t, u_img - floor_u_img);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    static cv::Vec<float, 3> interpolate1d(cv::Vec<float, 3>& a, cv::Vec<float, 3>& b, float alpha) {
        return a * (1 - alpha) + b * alpha;
    }

};
#endif //RASTERIZER_TEXTURE_H
