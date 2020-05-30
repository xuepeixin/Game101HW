//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = intersect(ray);
    if (!p.happened) return Vector3f(0,0,0);
    Vector3f wo = -ray.direction;


    if (p.obj->hasEmit()) {
        return p.emit;
    }
    return shade(p, wo);

}

Vector3f Scene::shade(const Intersection &p, const Vector3f &wo) const{
    Intersection inter;
    float pdf;
    sampleLight(inter, pdf);
    auto ws = (inter.coords-p.coords).normalized();
    auto N = p.normal;
    auto NN = inter.normal;
    Ray ray_l(p.coords, ws);
    Intersection block = intersect(ray_l);
    Vector3f L_dir = 0.f;
    if (block.happened && block.obj->hasEmit()) {
        L_dir = inter.emit * p.m->eval(wo, ws, N) * 
                dotProduct(ws, N) * dotProduct(-ws, NN) / 
                pow((inter.coords-p.coords).norm(), 2) / pdf;
    }

    Vector3f L_indir = 0.f;

    if (get_random_float() < RussianRoulette) {
        auto wi = p.m->sample(wo, N);
        Ray ray_r(p.coords, wi);
        block = intersect(ray_r);
        if (block.happened && !block.obj->hasEmit()) {
            L_indir = shade(block, wi) * p.m->eval(wo, wi, N) * dotProduct(wi, N) /
                        p.m->pdf(wo, wi, N) / RussianRoulette;
        }
    } 
    return L_dir + L_indir;
}

