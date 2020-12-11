//
// Created by hyin on 2020/3/25.
//

#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

#include <pcl/impl/point_types.hpp>
#include <vector>

struct Node {
    pcl::PointXYZI point;
    int id;
    Node *left;
    Node *right;

    Node(pcl::PointXYZI arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node *root;

    KdTree() : root(NULL) {}

    void insertHelper(Node **node, int depth, pcl::PointXYZI point, int id) {
        // Tree is empty
        if (*node == NULL) {
            *node = new Node{point, id};
        } else {
            // calculate current din
            // 交替比较x/y/z的大小，进行子节点的插入
            int cd = depth % 3;
            if (cd == 0) {
                if (point.x < (*node)->point.x) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            } else if (cd == 1) {
                if (point.y < (*node)->point.y) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            } else {
                if (point.z < (*node)->point.z) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
        }
    }

    void insert(pcl::PointXYZI point, int id) {
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(pcl::PointXYZI target, Node *node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            float delta_x = node->point.x - target.x;
            float delta_y = node->point.y - target.y;
            float delta_z = node->point.z - target.z;

            // 计算与target距离差值小于阈值的点，如果单维坐标差值已经超过，则直接跳过
            if ((delta_x >= -distanceTol && delta_x <= distanceTol) &&
                (delta_y >= -distanceTol && delta_y <= distanceTol) &&
                (delta_z >= -distanceTol && delta_z <= distanceTol)) {
                float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

                if (distance <= distanceTol) {
                    ids.push_back(node->id);        // ids保留经过的距离内搜索路径节点，这些点都在target附近
                }
            }
            // check across boundary
            // 由于插入是xyz交替比较进行，因此搜索也是按xyz交替比较
            if (depth % 3 == 0) {
                // 若node在target+-distanceTol的方框范围内，则意味着需要向对应的方向(小->左，大->右)继续搜索
                if (delta_x > -distanceTol) {       
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if (delta_x < distanceTol) {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            } else if (depth % 3 == 1) {
                if (delta_y > -distanceTol) {
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if (delta_y < distanceTol) {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            } else {
                if (delta_z > -distanceTol) {
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if (delta_z < distanceTol) {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            }

        }
    }
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }

};


#endif //PLAYBACK_KDTREE3D_H
