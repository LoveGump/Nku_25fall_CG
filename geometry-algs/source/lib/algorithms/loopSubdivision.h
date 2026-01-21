#ifndef LOOP_SUBDIVISION_H
#define LOOP_SUBDIVISION_H

#include <set>
#include <vector>
#include "MeshTopology.h"
#include "solverBase.h"

#ifndef NDEBUG
#include <iostream>
#endif


namespace LoopSubdivision 
{ // ref: https://pbr-book.org/3ed-2018/Shapes/Subdivision_Surfaces#SDVertex::regular
using namespace meshTopology;

static void loopSubdivision(tinyobj::attrib_t&, std::vector<tinyobj::shape_t>&);
inline real_t beta(int valence);
template <class V>
void computeWeightedPosition(const std::vector<V*> &comp, real_t beta, Point &resp);

class LoopSolver: public SolverBase
{
    const char *name = "Loop Subdivision";
public:
    const char *getName() {return name;}
    void run(tinyobj::attrib_t &attrib, std::vector<tinyobj::shape_t> &shapes) 
    {
        std::vector<SDShape> sdshapes;
        SDAttrib sdattrib;
        createSDShapes(attrib, shapes, sdattrib, sdshapes);

        loopSubdivision(sdattrib, sdshapes);

        finMtx.lock();
        hasFinished = true;
        sourceMtx.lock();
        retainNormalVector(sdattrib, sdshapes);
        createObjShapes(sdattrib, sdshapes, attrib, shapes);
        sourceMtx.unlock();
        finMtx.unlock();
    }

    /*
    * Perform Loop subdivision. Save the mesh after subdivision in sdattrib and sdshapes.
        normal vector and textcoord are not neeeded. In output, we just leave sdattrib.ns and sdattrib.ts empty.
    * input & output: sdattrib. sdattrib saves vertex, normal vector and textcoord of the mesh.
        sdshapes. sdshapes saves topology structure of the mesh.
    * ref: https://pbr-book.org/3ed-2018/Shapes/Subdivision_Surfaces
    */
    void loopSubdivision(SDAttrib &sdattrib, std::vector<SDShape> &sdshapes)
    {
        for (SDShape &shape : sdshapes)
        {
            // 第一步：更新旧顶点（偶顶点）
            // 内部顶点：v_new = (1 - n*beta) * v + beta * sum(neighbors)
            // 边界顶点：v_new = 3/4 * v0 + 1/8 * (v1 + v2)，其中v1,v2为边界上相邻顶点
            for (SDVertex *v : sdattrib.vs)
            {
                std::vector<SDVertex*> neighbors;
                findVtxNeighbors(v, std::back_inserter(neighbors));
                v->child = new SDVertex(v->p);
                if (isBoundary(v))
                {
                    // 边界顶点：取边界上的首尾两个邻居
                    SDVertex *v1 = neighbors.front();
                    SDVertex *v2 = neighbors.back();
                    v->child->p = v->p * 0.75f + (v1->p + v2->p) * 0.125f;
                }
                else
                {
                    computeWeightedPosition(neighbors, beta(v->valence), v->child->p);
                }
            }

            // 第二步：生成新顶点（奇顶点）
            // 遍历边集合，在每条边的中心位置生成细分点
            // 边界边：取中点 (v0 + v1) / 2
            // 内部边：加权 3/8*(v0+v1) + 1/8*(v2+v3)，其中v2,v3为对面顶点
            // 将新顶点存入 f->sdverts[edgeNum] 供后续面拆分使用
            for (const SDEdge &e : shape.es)
            {
                SDVertex *odd = new SDVertex();
                if (e.boundary())
                {
                    odd->p = (e.verts[0]->p + e.verts[1]->p) * 0.5f;
                }
                else
                {
                    SDVertex *v2 = e.fs[0]->otherVertex(&e);
                    SDVertex *v3 = e.fs[1]->otherVertex(&e);
                    odd->p = (e.verts[0]->p + e.verts[1]->p) * 0.375f + (v2->p + v3->p) * 0.125f;
                }
                odd->index = sdattrib.vs.size();
                sdattrib.vs.push_back(odd);
                e.fs[0]->sdverts[e.f0edgeNum] = odd;
                if (e.fs[1]) e.fs[1]->sdverts[e.f1edgeNum] = odd;
            }

            // 将计算好的偶顶点新位置写回原顶点
            for (SDVertex *v : sdattrib.vs)
            {
                if (v->child)
                {
                    v->child->index = v->index;
                    v->p = v->child->p;
                    delete v->child;
                    v->child = nullptr;
                }
            }

            // 第三步：重建拓扑结构（1拆4）
            // 每个原始三角形派生出4个子面：3个角部三角形 + 1个中心三角形
            //       v0                    v0
            //      /  \                  /  \
            //     /    \      =>       e2----e0
            //    /      \             /  \  /  \
            //   v2------v1          v2----e1----v1
            std::vector<SDFace*> newFaces;
            for (SDFace *f : shape.fs)
            {
                SDVertex *v0 = f->verts[0], *v1 = f->verts[1], *v2 = f->verts[2];
                SDVertex *e0 = f->sdverts[0], *e1 = f->sdverts[1], *e2 = f->sdverts[2];

                f->child[0]->verts = {v0, e0, e2};  // 角部三角形0
                f->child[1]->verts = {e0, v1, e1};  // 角部三角形1
                f->child[2]->verts = {e2, e1, v2};  // 角部三角形2
                f->child[3]->verts = {e0, e1, e2};  // 中心三角形

                for (int i = 0; i < 4; i++)
                {
                    f->child[i]->index = newFaces.size();
                    newFaces.push_back(f->child[i]);
                }
            }
            shape.fs.swap(newFaces);

            // 第四步：收尾 - 重建边集合和邻接关系
            // 更新 neighborFs 指针，维持网格连通性以支持连续细分
            shape.es.clear();
            for (SDVertex *v : sdattrib.vs) v->valence = 0;

            std::set<SDEdge> edges;
            for (SDFace *f : shape.fs)
            {
                for (int j = 0; j < 3; j++)
                {
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
                        // 更新邻接面指针
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

// β = 1/n * (5/8 - (3/8 + 1/4 * cos(2π/n))^2)
inline real_t beta(int n)
{
    real_t c = 3.f/8.f + 0.25f * cos(2.f * M_PI / n);
    return (5.f/8.f - c * c) / n;
}

template <class V>
void computeWeightedPosition(const std::vector<V*> &comp, real_t beta, Point &resp)
{
    for (int i = 0; i < 3; i++)
    {
        resp[i] *= (1 - comp.size() * beta);
        for (V *vertp: comp) 
            resp[i] += vertp->p[i] * beta;
    }
}


} // namespace: loopSubdivision

#endif