#pragma once
#ifndef __PHOTON_MAP_HPP__
#define __PHOTON_MAP_HPP__

#include <vector>
#include <memory>
#include <algorithm>
#include <cfloat>
#include "geometry/vec.hpp"
#include "Photon.hpp"

namespace SimplePathTracer {
    using namespace NRenderer;
    using std::vector;

    struct PhotonNode {
        // 使用中位划分的KD-Tree节点
        int axis;          // 分割轴: 0/1/2 -> x/y/z, -1 表示叶子
        int index;         // 当前节点对应的光子索引（在叶子或中间节点保存一个样本）
        int left;          // 左子节点索引，-1 表示无
        int right;         // 右子节点索引，-1 表示无
        Vec3 bboxMin;      // 包围盒最小角
        Vec3 bboxMax;      // 包围盒最大角
    };

    class PhotonMap {
    public:
        PhotonMap() = default;

        inline void build(const vector<Photon>* photons);
        inline void queryRadius(const Vec3& p, float radius, vector<const Photon*>& out) const;
        bool empty() const { return !photons_ || photons_->empty(); }

    private:
        const vector<Photon>* photons_ {nullptr};
        vector<int> indices_;
        vector<PhotonNode> nodes_;

        inline int buildRecursive(int begin, int end);
        inline void queryRecursive(int nodeId, const Vec3& p, float r2, vector<const Photon*>& out) const;
    };

    // -------- inline implementation ---------
    static inline int longestAxis(const Vec3& e) {
        if (e.x >= e.y && e.x >= e.z) return 0;
        if (e.y >= e.x && e.y >= e.z) return 1;
        return 2;
    }

    static inline float dist2PointAABB(const Vec3& p, const Vec3& bmin, const Vec3& bmax) {
        float dx = (p.x < bmin.x) ? (bmin.x - p.x) : (p.x > bmax.x ? p.x - bmax.x : 0.0f);
        float dy = (p.y < bmin.y) ? (bmin.y - p.y) : (p.y > bmax.y ? p.y - bmax.y : 0.0f);
        float dz = (p.z < bmin.z) ? (bmin.z - p.z) : (p.z > bmax.z ? p.z - bmax.z : 0.0f);
        return dx*dx + dy*dy + dz*dz;
    }

    inline void PhotonMap::build(const vector<Photon>* photons) {
        photons_ = photons;
        indices_.resize(photons_->size());
        for (size_t i=0;i<indices_.size();++i) indices_[i] = static_cast<int>(i);
        nodes_.clear();
        nodes_.reserve(photons_->size()*2);
        if (!indices_.empty()) {
            buildRecursive(0, static_cast<int>(indices_.size()));
        }
    }

    inline int PhotonMap::buildRecursive(int begin, int end) {
        int count = end - begin;
        if (count <= 0) return -1;

        Vec3 bmin{FLT_MAX, FLT_MAX, FLT_MAX};
        Vec3 bmax{-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (int i=begin;i<end;++i) {
            const Vec3& p = photons_->at(indices_[i]).position;
            bmin = glm::min(bmin, p);
            bmax = glm::max(bmax, p);
        }
        Vec3 extent = bmax - bmin;
        int axis = longestAxis(extent);

        int mid = begin + count/2;
        std::nth_element(indices_.begin()+begin, indices_.begin()+mid, indices_.begin()+end,
            [&](int a, int b){
                const Vec3& pa = photons_->at(a).position;
                const Vec3& pb = photons_->at(b).position;
                if (axis==0) return pa.x < pb.x;
                if (axis==1) return pa.y < pb.y;
                return pa.z < pb.z;
            }
        );

        int nodeId = static_cast<int>(nodes_.size());
        nodes_.push_back({axis, indices_[mid], -1, -1, bmin, bmax});
        if (mid-begin > 0) nodes_[nodeId].left = buildRecursive(begin, mid);
        if (end-(mid+1) > 0) nodes_[nodeId].right = buildRecursive(mid+1, end);
        return nodeId;
    }

    inline void PhotonMap::queryRadius(const Vec3& p, float radius, vector<const Photon*>& out) const {
        if (nodes_.empty()) return;
        float r2 = radius*radius;
        queryRecursive(0, p, r2, out);
    }

    inline void PhotonMap::queryRecursive(int nodeId, const Vec3& p, float r2, vector<const Photon*>& out) const {
        if (nodeId < 0) return;
        const PhotonNode& node = nodes_[nodeId];
        if (dist2PointAABB(p, node.bboxMin, node.bboxMax) > r2) return;

        const Photon& ph = photons_->at(node.index);
        Vec3 d = ph.position - p;
        float d2 = glm::dot(d, d);
        if (d2 <= r2) out.push_back(&ph);

        float splitCoord = (node.axis==0? ph.position.x : (node.axis==1? ph.position.y : ph.position.z));
        float pointCoord = (node.axis==0? p.x : (node.axis==1? p.y : p.z));

        int first = (pointCoord <= splitCoord) ? node.left : node.right;
        int second = (pointCoord <= splitCoord) ? node.right : node.left;
        if (first >= 0) queryRecursive(first, p, r2, out);
        float delta = pointCoord - splitCoord;
        if (second >= 0 && delta*delta <= r2) queryRecursive(second, p, r2, out);
    }
}

#endif
