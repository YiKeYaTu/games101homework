#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

//    root = recursiveBuild(primitives);
    root = recursiveBuildBySAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

static double computeArea(std::vector<Object*>::iterator begin, std::vector<Object*>::iterator end) {
    double xMin = std::numeric_limits<double>::max(), xMax = std::numeric_limits<double>::min();
    double yMin = xMin, yMax = xMax;
    double zMin = xMin, zMax = xMax;

    while (begin < end) {
        const auto& bounds =(*begin)->getBounds();

        if (bounds.pMin.x < xMin) {
            xMin = bounds.pMin.x;
        }
        if (bounds.pMin.y < yMin) {
            yMin = bounds.pMin.y;
        }
        if (bounds.pMin.z < zMin) {
            zMin = bounds.pMin.z;
        }
        if (bounds.pMax.x < xMin) {
            xMax = bounds.pMax.x;
        }
        if (bounds.pMax.y < yMin) {
            yMax = bounds.pMax.y;
        }
        if (bounds.pMax.z < zMin) {
            zMax = bounds.pMax.z;
        }

        begin ++;
    }

    return (xMax - xMin) * (yMax - yMin) * (zMax - zMin);
}

static int findPartitionIndex(std::vector<Object*>& objects, int dim) {
    int partitionIndex;
    double minByDim, maxByDim;

    switch (dim) {
        case 0:
            minByDim = objects.front()->getBounds().Centroid().x;
            maxByDim = objects.back()->getBounds().Centroid().x;
            break;
        case 1:
            minByDim = objects.front()->getBounds().Centroid().y;
            maxByDim = objects.back()->getBounds().Centroid().y;
            break;
        case 2:
            minByDim = objects.front()->getBounds().Centroid().z;
            maxByDim = objects.back()->getBounds().Centroid().z;
            break;
    }

    int numBucket = 16;
    double stride = (maxByDim - minByDim) / numBucket;
    std::vector<int> buckets(numBucket, 0);

    int bucketIndex;
    for (Object* object : objects) {
        switch (dim) {
            case 0:
                bucketIndex = (object->getBounds().Centroid().x - minByDim) / stride;
                break;
            case 1:
                bucketIndex = (object->getBounds().Centroid().y - minByDim) / stride;
                break;
            case 2:
                bucketIndex = (object->getBounds().Centroid().z - minByDim) / stride;
                break;
        }

        assert(bucketIndex <= numBucket);
        assert(bucketIndex >= 0);
        if (bucketIndex >= numBucket) {
            bucketIndex = numBucket - 1;
        }
        buckets[bucketIndex] ++;
    }

    double costLeft, costRight;
    double costTotal = std::numeric_limits<double>::max();
    double costPerPrimitive = 1.0;

    int preNumObjects = 0;
    double N = computeArea(objects.begin(), objects.end());

    for (int i = 0; i < numBucket - 1; ++i) {
        if (buckets[i] == 0) {
            continue;
        }

        preNumObjects += buckets[i];

        costLeft = preNumObjects * costPerPrimitive;
        costRight = (objects.size() - preNumObjects) * costPerPrimitive;

        double A = computeArea(objects.begin(), objects.begin() + preNumObjects);
        double B = computeArea(objects.begin() + preNumObjects, objects.end());

        if (costLeft + costRight < costTotal) {
            costTotal = A / N * costLeft + B / N * costRight;
            partitionIndex = preNumObjects;
        }
    }

    return partitionIndex;
}

BVHBuildNode* BVHAccel::recursiveBuildBySAH(std::vector<Object*> objects) {

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

        int partitionIndex = findPartitionIndex(objects, dim);

        auto beginning = objects.begin();
        auto middling = objects.begin() + partitionIndex;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildBySAH(leftshapes);
        node->right = recursiveBuildBySAH(rightshapes);

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
    std::array<int, 3> dirIsNeg { int(ray.direction.x < 0), int(ray.direction.y < 0), int(ray.direction.z < 0) };
    // TODO Traverse the BVH to find intersection
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        if (node->object) {
            auto intersection = node->object->getIntersection(ray);
            return intersection;
        }

        auto leftIntersection = getIntersection(node->left, ray);
        auto rightIntersection = getIntersection(node->right, ray);

        if (leftIntersection.happened && rightIntersection.happened) {
            if (leftIntersection.distance < rightIntersection.happened) {
                return leftIntersection;
            } else {
                return rightIntersection;
            }
        }

        if (leftIntersection.happened) {
            return leftIntersection;
        }

        if (rightIntersection.happened) {
            return rightIntersection;
        }
    }

    return Intersection();
}