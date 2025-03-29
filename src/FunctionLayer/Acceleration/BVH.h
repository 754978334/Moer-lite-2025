#pragma once
#include "Acceleration.h"
class BVH : public Acceleration{
public:
    enum class SplitMethod{ SAH, Middle, EqualCounts };
    BVH() = default;
    void build() override;
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;
protected:
    // static constexpr int bvhLeafMaxSize = 1;
    SplitMethod splitMethod = SplitMethod::EqualCounts;
    struct BVHNode;
    struct BVHShape;
    struct BVHSplitBucket;
    BVHNode * root;

    static const int LeafMaxSize = 64;
    BVHNode* buildRecursive(std::vector<BVHShape>& shapeInfos, int l, int r, std::vector<std::shared_ptr<Shape>>& orderedShapes);
    bool rayIntersectRecursive(BVHNode* node, Ray &ray, int *geomID, int *primID, float *u, float *v) const;
};