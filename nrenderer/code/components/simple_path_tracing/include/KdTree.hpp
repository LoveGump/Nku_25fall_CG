#pragma once
#ifndef __SIMPLE_PATH_TRACER_KDTREE_HPP__
#define __SIMPLE_PATH_TRACER_KDTREE_HPP__

#define SIMPLE_PT_MIN_BOX 10
#define SIMPLE_PT_MAX_OBJ 100

#include <algorithm>
#include <cmath>
#include <list>
#include <numeric>
#include <queue>
#include <vector>

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "glm/glm.hpp"

#define SIMPLE_PT_SPHERE 100
#define SIMPLE_PT_TRIANGLE 101

namespace SimplePathTracer
{
    struct Box
    {
        Box() = default;

        // 构造函数，初始化包围盒为球体或三角形
        // explicit 表示该构造函数不能用于隐式类型转换
        Box(NRenderer::Sphere *sp)
        {
            type = SIMPLE_PT_SPHERE;
            this->sp = sp;
            float r = sp->radius;
            min = {sp->position.x - r, sp->position.y - r, sp->position.z - r};
            max = {sp->position.x + r, sp->position.y + r, sp->position.z + r};
        }
        Box(NRenderer::Triangle *tr)
        {
            type = SIMPLE_PT_TRIANGLE;
            this->tr = tr;
            min = {std::min(tr->v1.x, std::min(tr->v2.x, tr->v3.x)),
                   std::min(tr->v1.y, std::min(tr->v2.y, tr->v3.y)),
                   std::min(tr->v1.z, std::min(tr->v2.z, tr->v3.z))};
            max = {std::max(tr->v1.x, std::max(tr->v2.x, tr->v3.x)),
                   std::max(tr->v1.y, std::max(tr->v2.y, tr->v3.y)),
                   std::max(tr->v1.z, std::max(tr->v2.z, tr->v3.z))};
        }
        union
        {
            NRenderer::Sphere *sp;
            NRenderer::Triangle *tr;
        };
        int type = -1;
        std::vector<float> min{3};
        std::vector<float> max{3};
    };

    // Kd树节点结构体
    struct KdNode
    {
        // 默认构造函数
        KdNode()
        {
            min = std::vector<float>(3, INFINITY);
            max = std::vector<float>(3, -INFINITY);
            left = right = nullptr;
        }
        // 构造函数，使用包围盒初始化节点
        KdNode(Box *box)
        {
            min = std::vector<float>(3, INFINITY);
            max = std::vector<float>(3, -INFINITY);
            update(box);
            left = right = nullptr;
            boxList.push_back(box);
        }
        // 构造函数，使用父节点和分割信息初始化节点
        KdNode(KdNode *n, int index, bool flag, int v)
        {
            min = std::vector<float>(3);
            max = std::vector<float>(3);
            for (int i = 0; i < min.size(); i++)
            {
                min[i] = n->min[i];
            }
            for (int i = 0; i < max.size(); i++)
            {
                max[i] = n->max[i];
            }
            if (flag)
            {
                max[index] = v;
            }
            else
            {
                min[index] = v;
            }
            left = right = nullptr;
        }
        // 更新节点的包围盒以包含新的包围盒
        void update(Box *box)
        {
            for (int i = 0; i < box->min.size(); i++)
            {
                min[i] = std::min(box->min[i], min[i]);
            }
            for (int i = 0; i < box->max.size(); i++)
            {
                max[i] = std::max(box->max[i], max[i]);
            }
        }

        //
        bool IsInNode(const Box *box)
        {
            for (int i = 0; i < box->min.size(); i++)
            {
                if (box->min[i] < min[i])
                    return false;
            }
            for (int i = 0; i < box->max.size(); i++)
            {
                if (box->max[i] > max[i])
                    return false;
            }
            return true;
        }
        void Insert(Box *box)
        {
            update(box);
            boxList.push_back(box);
        }
        std::vector<float> min;
        std::vector<float> max;
        std::list<Box *> boxList;
        KdNode *left = nullptr;
        KdNode *right = nullptr;
        bool isLeaf = true;
        float r;
    };

    struct KdTree
    {
        KdTree()
        {
            root = new KdNode;
        }

        void Insert(std::vector<Box *> &boxs, int s, int e, KdNode *n)
        {
            for (int i = s; i < e; i++)
            {
                n->update(boxs[i]);
            }
            if (s == e)
                return;
            else if (e - s <= maxSize)
            {
                for (int i = s; i < e; i++)
                {
                    n->Insert(boxs[i]);
                }
            }
            else
            {
                for (int i = s; i < e; i++)
                {
                    n->update(boxs[i]);
                }
                n->isLeaf = false;
                n->left = new KdNode;
                n->right = new KdNode;
                std::sort(boxs.begin() + s, boxs.begin() + e, [&](const Box *A, const Box *B)
                          {
                    if (axis == 1) {
                        return A->min[0] < B->min[0];
                    }
                    else if (axis == 2) {
                        return A->min[1] < B->min[1];
                    }
                    else {
                        return A->min[2] < B->min[2];
                    } });
                axis = (axis + 1) % 3;
                int mid = (s + e) / 2;
                Insert(boxs, s, mid, n->left);
                Insert(boxs, mid, e, n->right);
            }
        }
        void insert(std::vector<NRenderer::Sphere> &spheres, std::vector<NRenderer::Triangle> &triangles)
        {
            std::vector<Box *> boxes;
            boxes.reserve(spheres.size() + triangles.size());
            for (auto &s : spheres)
                boxes.push_back(new Box(&s));
            for (auto &t : triangles)
                boxes.push_back(new Box(&t));
            insert(boxes, 0, static_cast<int>(boxes.size()), root);
        }

        std::list<KdNode *> findNode(const Ray &r)
        {
            std::list<KdNode *> result;
            findNode(r, root, result);
            return result;
        }

    private:
        void insert(std::vector<Box *> &boxes, int s, int e, KdNode *node)
        {
            if (s >= e)
                return;
            if (node == nullptr)
                return;

            for (int i = s; i < e; ++i)
                node->update(boxes[i]);
            if (e - s <= maxSize)
            {
                for (int i = s; i < e; ++i)
                    node->boxList.push_back(boxes[i]);
                return;
            }

            node->isLeaf = false;
            node->left = new KdNode;
            node->right = new KdNode;
            std::sort(boxes.begin() + s, boxes.begin() + e, [&](const Box *A, const Box *B)
                      { return A->min[axis] < B->min[axis]; });
            axis = (axis + 1) % 3;
            int mid = (s + e) / 2;
            insert(boxes, s, mid, node->left);
            insert(boxes, mid, e, node->right);
            if (node->left)
            {
                node->min[0] = std::min(node->min[0], node->left->min[0]);
                node->min[1] = std::min(node->min[1], node->left->min[1]);
                node->min[2] = std::min(node->min[2], node->left->min[2]);
                node->max[0] = std::max(node->max[0], node->left->max[0]);
                node->max[1] = std::max(node->max[1], node->left->max[1]);
                node->max[2] = std::max(node->max[2], node->left->max[2]);
            }
            if (node->right)
            {
                node->min[0] = std::min(node->min[0], node->right->min[0]);
                node->min[1] = std::min(node->min[1], node->right->min[1]);
                node->min[2] = std::min(node->min[2], node->right->min[2]);
                node->max[0] = std::max(node->max[0], node->right->max[0]);
                node->max[1] = std::max(node->max[1], node->right->max[1]);
                node->max[2] = std::max(node->max[2], node->right->max[2]);
            }
        }

        bool isHit(const Ray &r, KdNode *node)
        {
            if (!node)
                return false;
            auto inv_dir = 1.0f / r.direction;
            NRenderer::Vec3 cube_min(node->min[0], node->min[1], node->min[2]);
            NRenderer::Vec3 cube_max(node->max[0], node->max[1], node->max[2]);
            auto tMin = (cube_min - r.origin) * inv_dir;
            auto tMax = (cube_max - r.origin) * inv_dir;
            auto t1 = glm::min(tMin, tMax);
            auto t2 = glm::max(tMin, tMax);
            float tNear = std::max(std::max(t1.x, t1.y), t1.z);
            float tFar = std::min(std::min(t2.x, t2.y), t2.z);
            return tNear < tFar;
        }

        void findNode(const Ray &r, KdNode *node, std::list<KdNode *> &result)
        {
            if (!node)
                return;
            if (!isHit(r, node))
                return;
            if (node->isLeaf)
            {
                result.push_back(node);
            }
            else
            {
                findNode(r, node->left, result);
                findNode(r, node->right, result);
            }
        }

        KdNode *root = nullptr;
        int maxSize = 3;
        int axis = 0; // 分割轴
    };

}

#endif
