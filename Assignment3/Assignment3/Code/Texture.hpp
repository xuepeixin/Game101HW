//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::min(1.0f, u);
        v = std::min(1.0f, v);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        // auto color_u_floor = image_data.at<cv::Vec3b>(floor(v_img), floor(u_img)) + 
        //                      (u_img-int(u_img)) * (image_data.at<cv::Vec3b>(floor(v_img), ceil(u_img)) - 
        //                                            image_data.at<cv::Vec3b>(floor(v_img), floor(u_img)));
        // auto color_u_ceil = image_data.at<cv::Vec3b>(ceil(v_img), floor(u_img)) + 
        //                     (u_img-int(u_img)) * (image_data.at<cv::Vec3b>(ceil(v_img), ceil(u_img)) - 
        //                                           image_data.at<cv::Vec3b>(ceil(v_img), floor(u_img)));

        // auto color = color_u_floor + (v_img-int(v_img)) * (color_u_ceil - color_u_floor);

        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
