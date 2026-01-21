#ifndef ROOT3_SUBDIVISION_H
#define ROOT3_SUBDIVISION_H

#include <cmath>
#include "solverBase.h"
#include "MeshTopology.h"

#ifndef NDEBUG
#include <iostream>
#endif

namespace root3Subdivision
{
    using namespace meshTopology;

    static void root3Subdivision(tinyobj::attrib_t &, std::vector<tinyobj::shape_t> &);
    template <class It>
    inline void oneRingRule(const It begin, const It end, SDVertex *res, real_t w);
    inline real_t alpha(int valence);

    class Root3Solver : public SolverBase
    {
        const char *name = "Root3 Subdivision";

    public:
        const char *getName() { return name; }
        void run(tinyobj::attrib_t &attrib, std::vector<tinyobj::shape_t> &shapes)
        {
            std::vector<SDShape> sdshapes;
            SDAttrib sdattrib;
            createSDShapes(attrib, shapes, sdattrib, sdshapes);

            root3Subdivision(sdattrib, sdshapes);

            finMtx.lock();
            hasFinished = true;
            sourceMtx.lock();
            retainNormalVector(sdattrib, sdshapes);
            createObjShapes(sdattrib, sdshapes, attrib, shapes);
            sourceMtx.unlock();
            finMtx.unlock();
        }

        /*
        * Perform Root3 subdivision. Save the mesh after subdivision in sdattrib and sdshapes.
            normal vector and textcoord are not needed. In output, we just leave sdattrib.ns and sdattrib.ts empty.
        * input & output: sdattrib. sdattrib saves vertex, normal vector and textcoord of the mesh.
            sdshapes. sdshapes saves topology structure of the mesh.
        * ref: https://dl.acm.org/doi/10.1145/344779.344835
        */
        void root3Subdivision(SDAttrib &sdattrib, std::vector<SDShape> &sdshapes)
        {
            for (SDShape &shape : sdshapes)
            {
                size_t origVertCount = sdattrib.vs.size();

                // 第一步：生成奇顶点（面重心）
                // 对每个三角形，新顶点位置为 V = 1/3 * (v1 + v2 + v3)
                for (SDFace *f : shape.fs)
                {
                    Point centroid = (f->verts[0]->p + f->verts[1]->p + f->verts[2]->p) / 3.f;
                    SDVertex *newv = new SDVertex(centroid);
                    newv->index = sdattrib.vs.size();
                    sdattrib.vs.push_back(newv);
                    f->sdverts[0] = newv;
                }

                // 第二步：更新偶顶点
                // v' = (1 - α_n) * v + α_n * (1/n) * Σu，其中 α_n = (4 - 2cos(2π/n)) / 9, n = 顶点的度
                for (size_t i = 0; i < origVertCount; i++)
                {
                    SDVertex *v = sdattrib.vs[i];
                    std::vector<SDVertex *> neighbors;
                    findAllNeighbors(v, std::back_inserter(neighbors));
                    int n = v->valence;
                    real_t a = alpha(n);
                    Point avg(0, 0, 0);
                    for (SDVertex *u : neighbors)
                        avg = avg + u->p;
                    avg = avg / (real_t)n;
                    v->child = new SDVertex((1.f - a) * v->p + a * avg);
                }

                // 应用偶顶点更新
                for (size_t i = 0; i < origVertCount; i++)
                {
                    SDVertex *v = sdattrib.vs[i];
                    v->p = v->child->p;
                    delete v->child;
                    v->child = nullptr;
                }

                // 第三步：重建拓扑 - 遍历原始边
                // 内部边：取出该边两侧面的中心点，与端点重新连接生成两个新面
                // 边界边：保持边界不被旋转
                std::vector<SDFace *> newFaces;
                for (auto it = shape.es.begin(); it != shape.es.end(); ++it)
                {
                    const SDEdge &e = *it;
                    SDVertex *c0 = e.fs[0]->sdverts[0];

                    SDVertex *v_start = e.fs[0]->verts[e.f0edgeNum];
                    SDVertex *v_end = e.fs[0]->verts[NEXT(e.f0edgeNum)];

                    if (e.boundary())
                    {
                        // 边界边：保持原边，连接到唯一的面中心
                        newFaces.push_back(new SDFace(v_start, v_end, c0));
                    }
                    else
                    {
                        // 内部边：连接两侧面中心点与边端点，生成两个新面（边翻转）
                        SDVertex *c1 = e.fs[1]->sdverts[0];
                        newFaces.push_back(new SDFace(v_start, c1, c0));
                        newFaces.push_back(new SDFace(v_end, c0, c1));
                    }
                }

                // 第四步：数据交换 - 重建拓扑结构
                shape.fs.swap(newFaces);
                for (SDFace *f : newFaces)
                    delete f; // 删除旧面
                newFaces.clear();
                shape.es.clear();
                for (SDVertex *v : sdattrib.vs)
                    v->valence = 0;

                std::set<SDEdge> edges;
                for (size_t fi = 0; fi < shape.fs.size(); fi++)
                {
                    SDFace *f = shape.fs[fi];
                    f->index = fi;
                    for (int j = 0; j < 3; j++)
                    {
                        f->neighborFs[j] = nullptr;
                        f->verts[j]->startFace = f;
                        SDEdge e(f->verts[j], f->verts[NEXT(j)]);
                        auto it = edges.find(e);
                        if (it == edges.end())
                        {
                            e.verts[0]->valence++;
                            e.verts[1]->valence++;
                            e.fs[0] = f;
                            e.f0edgeNum = j;
                            edges.insert(e);
                        }
                        else
                        {
                            it->fs[0]->neighborFs[it->f0edgeNum] = f;
                            f->neighborFs[j] = it->fs[0];
                            SDEdge newE = *it;
                            newE.fs[1] = f;
                            newE.f1edgeNum = j;
                            edges.erase(it);
                            edges.insert(newE);
                        }
                    }
                }
                shape.es = edges;
            }
        }
    };

    template <class It>
    inline void oneRingRule(const It begin, const It end, SDVertex *res, real_t w)
    {
        real_t wres = 1 - res->valence * w;
        for (int i = 0; i < 3; i++)
            res->p[i] *= wres;
        for (It ele = begin; ele != end; ++ele)
        {
            for (int i : {0, 1, 2})
                res->p[i] += w * (*ele)->p[i];
        }
    }

    // α_n = (4 - 2cos(2π/n)) / 9
    inline real_t alpha(int n)
    {
        return (4.f - 2.f * std::cos(2.f * M_PI / n)) / 9.f;
    }

}

#endif