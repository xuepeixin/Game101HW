// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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


static bool insideTriangle(float x, float y, const std::array<Vector4f, 3>& _v)
{   
    bool flag = true;
    bool pre_flag = true;
    for (int i = 0; i < 3; i++) {
        float area  = (x - _v[i][0])*(_v[(i+1)%3][1]-_v[i][1]) - 
                      (y - _v[i][1])*(_v[(i+1)%3][0]-_v[i][0]);
        if (area < 0)
            flag = false;
        else 
            flag = true;

        if (i != 0 && flag != pre_flag) return false;
        pre_flag = flag; 
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

void  getDepth(float x, float y, const Triangle& t, float* z_interpolated) {
    auto v = t.toVector4();
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    *z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    *z_interpolated *= w_reciprocal;
}

void subPixelCount(float x, float y, const Triangle& t, int* count) {
    auto v = t.toVector4();
    std::vector<std::vector<float>> delta = {{-0.5, -0.5}, {-0.5, 0.5},  {0.5, -0.5},  {0.5, 0.5}};
    *count = 0;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (insideTriangle(x+delta[j+2*i][0], y+delta[j+2*i][1],v) == true) {
                (*count)++;
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();

    for (auto p:v) {
        min_x = std::min(p[0], min_x);
        min_y = std::min(p[1], min_y);
        max_x = std::max(p[0], max_x);
        max_y = std::max(p[1], max_y);
    }

    min_x = std::max(0, int(min_x));
    min_y = std::max(0, int(min_y));
    max_x = std::min(this->width, int(max_x));
    max_y = std::min(this->width, int(max_y));



    // // 没有反走样

    // bool inside_table[this->width][this->height] = {false};
    // for (int x = std::floor(min_x); x < std::ceil(max_x); x++) {
    //     for (int y = std::floor(min_y); y < std::ceil(max_y); y++) {
    //         inside_table[x][y] = insideTriangle(x, y, v);
    //     }
    // }
    // for (int x = std::floor(min_x); x < std::ceil(max_x); x++) {
    //     for (int y = std::floor(min_y); y < std::ceil(max_y); y++) {
    //         if (inside_table[x][y]) {
    //             float z_interpolated;
    //             getDepth(x, y, t, &z_interpolated);
    //             if (z_interpolated < this->depth_buf[y+x*this->height]) {
    //                 this->depth_buf[y+x*this->height] = z_interpolated;
    //                 set_pixel(Vector3f(x, y, z_interpolated), t.getColor());
    //             }
    //         }
    //     }
    // }

    // 反走样，不使用sample_depth_buf
    
    bool inside_table[this->width][this->height] = {false};
    for (int x = std::floor(min_x); x < std::ceil(max_x); x++) {
        for (int y = std::floor(min_y); y < std::ceil(max_y); y++) {
            inside_table[x][y] = insideTriangle(x, y, v);
        }
    }
    std::vector<std::vector<float>> delta = {{-0.5, -0.5}, {-0.5, 0.5},  {0.5, -0.5},  {0.5, 0.5}};
    for (int x = std::floor(min_x); x < std::ceil(max_x); x++) {
        for (int y = std::floor(min_y); y < std::ceil(max_y); y++) {
            int count;
            float z_interpolated;
            if (y > 0 &&  inside_table[x][y-1] == false && inside_table[x][y] == true) {
                subPixelCount(x, y, t, &count);
                getDepth(x, y, t, &z_interpolated);
                if (z_interpolated < this->depth_buf[y+x*this->height]) {
                    this->depth_buf[get_index(x,y)] = z_interpolated;
                    auto background = get_pixel(Vector3f(x, y, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y, z_interpolated), mix_color);
                } else if (count > 0) {
                    auto background = get_pixel(Vector3f(x, y, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y, z_interpolated), mix_color);
                }

                subPixelCount(x, y-1, t, &count);
                getDepth(x, y-1, t, &z_interpolated);                
                
                if (count > 0) {
                    auto background = get_pixel(Vector3f(x, y-1, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y-1, z_interpolated/count), mix_color);
                }

            } else if (y < height-1 &&  inside_table[x][y] == true && inside_table[x][y+1] == false) {
                subPixelCount(x, y, t, &count);
                getDepth(x, y, t, &z_interpolated);                
                if (z_interpolated < this->depth_buf[get_index(x,y)]) {
                    this->depth_buf[get_index(x,y)] = z_interpolated;
                    auto background = get_pixel(Vector3f(x, y, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y, z_interpolated), mix_color);
                } else if (count > 0) {
                    auto background = get_pixel(Vector3f(x, y, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y, z_interpolated), mix_color);
                }

                subPixelCount(x, y+1, t, &count);
                getDepth(x, y+1, t, &z_interpolated);                
                if (count > 0) {
                    auto background = get_pixel(Vector3f(x, y+1, z_interpolated));
                    auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
                    set_pixel(Vector3f(x, y+1, z_interpolated/count), mix_color);
                }
            } else if (inside_table[x][y] == true) {
                getDepth(x, y, t, &z_interpolated);
                if (z_interpolated < this->depth_buf[get_index(x,y)]) {
                    this->depth_buf[get_index(x,y)] = z_interpolated;
                    set_pixel(Vector3f(x, y, z_interpolated), t.getColor());
                }
            }

        }
    }

    // // 使用sample_depth_buf反走样
    // bool inside_table[this->width*2][this->height*2] = {false};

    // for (int x = std::floor(min_x); x < std::ceil(max_x); x++) {
    //     for (int y = std::floor(min_y); y < std::ceil(max_y); y++) {
    //         inside_table[x*2][y*2] = insideTriangle(x-0.5, y-0.5, v);
    //         inside_table[x*2+1][y*2] = insideTriangle(x+0.5, y-0.5, v);
    //         inside_table[x*2][y*2+1] = insideTriangle(x-0.5, y+0.5, v);
    //         inside_table[x*2+1][y*2+1] = insideTriangle(x+0.5, y+0.5, v);
    //     }
    // }

    // for (int x = 0; x < this->width; x++) {
    //     for (int y = 0; y < this->height; y++) {
    //         std::vector<std::vector<float>> delta = {{-0.5, -0.5}, {-0.5, 0.5},  {0.5, -0.5},  {0.5, 0.5}};
    //         int count = 0;
    //         float total_z_interpolated = 0;
    //         for (int i = 0; i < 2; i++) {
    //             for (int j = 0; j < 2; j++) {
    //                 if (inside_table[x*2+i][y*2+j] == true) {
    //                     auto[alpha, beta, gamma] = computeBarycentric2D(x+delta[j+2*i][0], y+delta[j+2*i][1], t.v);
    //                     float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                     z_interpolated *= w_reciprocal;
    //                     total_z_interpolated += z_interpolated;
    //                     if (z_interpolated < this->sample_depth_buf[2*y+j+(2*x+i)*2*this->height]) {
    //                         this->sample_depth_buf[2*y+j+(2*x+i)*2*this->height] = z_interpolated;
    //                         count++;
    //                     }
    //                 }
                    
    //             }
    //         }
    //         if (count > 0) {
    //             auto background = get_pixel(Vector3f(x, y, total_z_interpolated/count));
    //             auto mix_color =(1-count / 4.0)* background + count / 4.0 * t.getColor();
    //             // std::cout << mix_color << "\n" << std::endl;
    //             set_pixel(Vector3f(x, y, total_z_interpolated/count), mix_color);
    //         }
    //     }
    // }

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle


    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    std::fill(sample_depth_buf.begin(), sample_depth_buf.end(),std::numeric_limits<float>::infinity());
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    sample_depth_buf.resize(w*h*2*2, std::numeric_limits<float>::infinity());
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

Eigen::Vector3f rst::rasterizer::get_pixel(const Eigen::Vector3f& point) 
{
    auto ind = (height-1-point.y())*width + point.x();
    return frame_buf[ind];
}

// clang-format on