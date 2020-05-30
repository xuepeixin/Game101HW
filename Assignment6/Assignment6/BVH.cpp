#include <algorithm>
#include <cassert>
#include "BVH.hpp"

Bucket::~Bucket() {
    delete this->bound_bucket_;
    for (int i = 0; i < 32; i++)
        delete this->bounds_[i];
}

void Bucket::addPrimitive(Bounds3 bound) {
    
    auto centroid = bound.Centroid();
    float value;

    int idx = int((centroid[0] - left_bound_) / (right_bound_ - left_bound_) * 32);
    idx = idx < 0 ? 0 : idx;
    idx = idx > 31 ? 31 : idx;
    if (bucket_count_[idx] == 0) {
        bounds_[idx] = new Bounds3(bound);
        bucket_count_[idx]++;
    } else {
        *(bounds_[idx]) = Union(*(bounds_[idx]), bound);
    }

    if (bound_bucket_ == nullptr) {
        bound_bucket_ = new Bounds3(bound);
    } else {
        *bound_bucket_ = Union(*bound_bucket_, bound);
    }
}

void Bucket::getBestPartition(int* best_idx, float* cost_min) {
    *cost_min = std::numeric_limits<float>::max();
    *best_idx = -1;
    for (int i = 0; i < 31; i++) {
        Bounds3* bound_left;
        Bounds3* bound_right;
        int count_left = 0;
        int count_right = 0;
        for (int j = 0; j <= i; j++) {
            if (bound_left != nullptr && bounds_[j] != nullptr) {
                *bound_left = Union(*bound_left, *bounds_[j]);
                count_left += bucket_count_[j];
            } else if (bound_left == nullptr && bounds_[j] != nullptr) {
                bound_left = new Bounds3(*bounds_[j]);
                count_left += bucket_count_[j];
            }
        }

        for (int j = i+1; j < 32; j++) {
            if (bound_right != nullptr && bounds_[j] != nullptr) {
                *bound_right = Union(*bound_right, *bounds_[j]);
                count_right += bucket_count_[j];
            } else if (bound_right == nullptr && bounds_[j] != nullptr) {
                bound_right = new Bounds3(*bounds_[j]);
                count_right += bucket_count_[j];
            }
        }
        if (bound_left != nullptr && bound_right != nullptr) {
            auto area_left = bound_left->SurfaceArea();
            auto area_right = bound_right->SurfaceArea();
            auto area =  bound_bucket_->SurfaceArea();
            float cost = area_left / area * count_left + area_right / area * count_right + 0.125;
            if (cost < *cost_min) {
                *cost_min = cost;
                *best_idx = i;
            } 
        }

        if (bound_left != nullptr) delete bound_left;
        if (bound_right != nullptr) delete bound_right;
    }
}
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    if (splitMethod == SplitMethod::NAIVE) {
        root = recursiveBuild(primitives);
    } else if (splitMethod == SplitMethod::SAH) {
        root = recursiveBuildSAH(primitives);
    }
    

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*>objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildSAH(std::vector{objects[0]});
        node->right = recursiveBuildSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {
        float best_partition;
        float best_cost = std::numeric_limits<float>::max();
        float axis;
        for (int cur_axis = 0; cur_axis < 3; cur_axis++) {
            double min_t = std::numeric_limits<double>::max();
            double max_t = std::numeric_limits<double>::max();
            for (int i = 0; i < objects.size(); i++) {
                min_t = objects[i]->getBounds().pMin[cur_axis] < min_t ? objects[i]->getBounds().pMin[cur_axis] : min_t;
                max_t = objects[i]->getBounds().pMax[cur_axis] > max_t ? objects[i]->getBounds().pMax[cur_axis] : max_t;
            }
            Bucket bucket(min_t, max_t, cur_axis);
            for (int i = 0; i < objects.size(); i++) {
                bucket.addPrimitive(objects[i]->getBounds());
            } 
            int idx;
            float cost;
            bucket.getBestPartition(&idx, &cost);
            float partition = min_t + idx *  (max_t - min_t) / 32;
            if (cost < best_cost) {
                best_cost = cost;
                axis = cur_axis;
                best_partition = partition;
            }
        }


        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;

        
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i]->getBounds().Centroid()[axis] < best_partition) {
                leftshapes.push_back(objects[i]);
            } else {
                rightshapes.push_back(objects[i]);
            }
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if (node == nullptr) return inter;

    bool flag = node->bounds.IntersectP(ray, 
                                        ray.direction_inv, 
                                        std::array<int, 3>({int(ray.direction[0] > 0), 
                                                            int(ray.direction[1] > 0), 
                                                            int(ray.direction[2] > 0)}));

    if (flag) {
        if (node->left == nullptr && node->right == nullptr) {
            return node->object->getIntersection(ray);
        } else {
            Intersection left_inter = BVHAccel::getIntersection(node->left, ray);
            Intersection right_inter = BVHAccel::getIntersection(node->right, ray);
            inter = left_inter.distance < right_inter.distance ? left_inter : right_inter;
            return inter;
        }
    } 

    return inter;
    

}