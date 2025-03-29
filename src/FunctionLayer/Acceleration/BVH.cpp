#include "BVH.h"
struct  BVH::BVHNode{
    BVHNode *left = NULL;
    BVHNode *right = NULL;
    AABB box{};
    int firstShapeOffset = 0;   // 叶子结点第一个物体的索引
    int nShape = 0;             // 叶子节点存了多少物体
    int splitAxis = -1;         // 非叶子节点的分割轴

    bool isLeaf() const { return splitAxis == -1; }
};

// 用于构建BVH中，对shape排序
struct BVH::BVHShape{
    std::shared_ptr<Shape> shape;
    size_t id;                     // 物体在shapes中的原索引
};

// SAH算法中，用于存储每个桶的信息
struct BVH::BVHSplitBucket{
    int count = 0;                // 图元数量  
    AABB box{};                   // 包围盒
};

void BVH::build() {
    std::vector<BVHShape> shapeInfos(shapes.size());
    std::vector<std::shared_ptr<Shape>> orderedShapes{};
    orderedShapes.reserve(shapes.size());
    for(int i = 0; i < shapes.size(); ++i)
    {
        //* 确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        const auto& shape = shapes[i];
        shape->initInternalAcceleration();
        boundingBox.Expand(shape->getAABB());

        shapeInfos[i] = {shape, (size_t)i};
    }
    
    root = buildRecursive(shapeInfos, 0, shapes.size(), orderedShapes);

    // override shapes with orderedShapes 
    shapes = std::move(orderedShapes);
}
bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    return rayIntersectRecursive(root, ray, geomID, primID, u, v);
}

BVH::BVHNode* BVH::buildRecursive(std::vector<BVHShape>& shapeInfos, int l, int r, std::vector<std::shared_ptr<Shape>>& orderedShapes){
    
    BVHNode* node = new BVHNode;
    for(int i = l; i < r; i++){
        node->box.Expand(shapeInfos[i].shape->getAABB());
    }

    // Leaf node
    if(r - l <= LeafMaxSize){
        node->firstShapeOffset = l;
        node->nShape = r - l;
        for(int i = l; i < r; i++){
            orderedShapes.push_back(shapeInfos[i].shape);
            node->box.Expand(shapeInfos[i].shape->getAABB());
        }
        return node;
    }

    AABB centroidBounds;
    for(int i = l; i < r; i++){
        centroidBounds.Expand(shapeInfos[i].shape->getAABB().Center());
    }
    int splitAxis = static_cast<int>(centroidBounds.MaxDimension());
    node->splitAxis = splitAxis;

    // split and recursively build
    int mid = (l + r) / 2;
    switch (splitMethod)
    {
        case SplitMethod::Middle:
        {
            float pmid = (centroidBounds.pMin[splitAxis] + centroidBounds.pMax[splitAxis]) / 2;
            auto midIter = std::partition(shapeInfos.begin() + l, shapeInfos.begin() + r, [splitAxis, pmid](const BVHShape& a){
                return a.shape->getAABB().Center()[splitAxis] < pmid;
            });
            mid = midIter - shapeInfos.begin();
            if(midIter != shapeInfos.begin() + l && midIter != shapeInfos.begin() + r)
                break;  // valid split
        }
        case SplitMethod::EqualCounts:
        {
            mid = (l + r) / 2;
            std::nth_element(shapeInfos.begin() + l, shapeInfos.begin() + mid, shapeInfos.begin() + r, [splitAxis](const BVHShape& a, const BVHShape& b){
                return a.shape->getAABB().Center()[splitAxis] < b.shape->getAABB().Center()[splitAxis];
            });
            break;
        }
        case SplitMethod::SAH:
        default:
        {
            if(r - l <= 2) {
                // too few primitives, use EqualCounts
                mid = (l + r) / 2;
                std::nth_element(shapeInfos.begin() + l, shapeInfos.begin() + mid, shapeInfos.begin() + r, [splitAxis](const BVHShape& a, const BVHShape& b){
                    return a.shape->getAABB().Center()[splitAxis] < b.shape->getAABB().Center()[splitAxis];
                });
            } else {
                // Allocate buckets and initialize
                constexpr int nBuckets = 12;
                std::array<BVHSplitBucket, nBuckets> buckets;
                for(int i = l; i < r; ++i){
                    const auto &prim = shapeInfos[i];
                    float up = prim.shape->getAABB().Center()[splitAxis] - centroidBounds.pMin[splitAxis];
                    float down = centroidBounds.pMax[splitAxis] - centroidBounds.pMin[splitAxis];
                    int b = std::clamp(static_cast<int>(up / down * nBuckets), 0, nBuckets - 1);
                    buckets[b].count++;
                    buckets[b].box.Expand(prim.shape->getAABB());
                }

                // Allocate cost arrays
                constexpr int nSplits = nBuckets - 1;
                std::array<float, nSplits> costs = {};

                // Compute costs
                int countBelow = 0;
                AABB boxBelow = AABB();
                for(int i = 0; i < nSplits; ++i) {
                    boxBelow.Expand(buckets[i].box);
                    countBelow += buckets[i].count;
                    costs[i] += boxBelow.SurfaceArea() * countBelow;
                }

                int countAbove = 0;
                AABB boxAbove = AABB();
                for(int i = nSplits; i >= 1; --i) {
                    boxAbove.Expand(buckets[i].box);
                    countAbove += buckets[i].count;
                    costs[i - 1] += boxAbove.SurfaceArea() * countAbove;
                }

                // Find best split
                int minCostSplitBucket = -1;
                float minCost = std::numeric_limits<float>::max();
                for(int i = 0; i < nSplits; ++i) {
                    if(costs[i] < minCost) {
                        minCost = costs[i];
                        minCostSplitBucket = i;
                    }
                }

                // Split
                auto midIter = std::partition(shapeInfos.begin() + l, shapeInfos.begin() + r, [=](const BVHShape& a) {
                    float up = a.shape->getAABB().Center()[splitAxis] - centroidBounds.pMin[splitAxis];
                    float down = centroidBounds.pMax[splitAxis] - centroidBounds.pMin[splitAxis];
                    int b = std::clamp(static_cast<int>(up / down * nBuckets), 0, nBuckets - 1);
                    return b <= minCostSplitBucket;
                });
                mid = midIter - shapeInfos.begin();

                // compute leaf cost and choose leaf if it's better
                // float leafCost = shapeInfos.size();
                // minCost = 1.0f / 2.0f + minCost / node->box.SurfaceArea();

                // if(shapeInfos.size() > LeafMaxSize || minCost < leafCost)
                // {
                //     // Split
                //     auto midIter = std::partition(shapeInfos.begin() + l, shapeInfos.begin() + r, [=](const BVHShape& a) {
                //         float up = a.shape->getAABB().Center()[splitAxis] - centroidBounds.pMin[splitAxis];
                //         float down = centroidBounds.pMax[splitAxis] - centroidBounds.pMin[splitAxis];
                //         int b = std::clamp(static_cast<int>(up / down * nBuckets), 0, nBuckets - 1);
                //         return b <= minCostSplitBucket;
                //     });
                //     mid = midIter - shapeInfos.begin();
                // } else {
                //     // Leaf
                //     node->firstShapeOffset = l;
                //     node->nShape = r - l;
                //     for(int i = l; i < r; i++){
                //         orderedShapes.push_back(shapeInfos[i].shape);
                //         node->box.Expand(shapeInfos[i].shape->getAABB());
                //     }
                //     return node;
                // }
            }
            break;
        }
    }

    node->left = buildRecursive(shapeInfos, l, mid, orderedShapes);
    node->right = buildRecursive(shapeInfos, mid, r, orderedShapes);

    return node;
}

bool BVH::rayIntersectRecursive(BVH::BVHNode* node, Ray &ray, int *geomID, int *primID, float *u, float *v) const
{
    // AABB intersection
    if(!node->box.RayIntersect(ray)) return false;

    // Leaf node
    if(node->isLeaf())
    {
        bool hit = false;
        for(int i = node->firstShapeOffset; i < node->firstShapeOffset + node->nShape; ++i)
        {
            if(shapes[i]->rayIntersectShape(ray, primID, u, v)){
                hit = true;
                *geomID = shapes[i]->geometryID;
            }
        }
        return hit;
    }

    // not Leaf node
    // bool hitLeft = recursiveRayIntersect(node->left, ray, geomID, primID, u, v);
    // bool hitRight = recursiveRayIntersect(node->right, ray, geomID, primID, u, v);
    // return hitLeft || hitRight;
    
    if(ray.direction[node->splitAxis] > 0.f){
        if(rayIntersectRecursive(node->left, ray, geomID, primID, u, v)) return true;
        return rayIntersectRecursive(node->right, ray, geomID, primID, u, v);
    }
    else {
        if(rayIntersectRecursive(node->right, ray, geomID, primID, u, v)) return true;
        return rayIntersectRecursive(node->left, ray, geomID, primID, u, v);
    }
    
}