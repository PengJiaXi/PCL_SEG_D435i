//
// Created by hyin on 2020/3/25.
//

#include "pcl_seg/cluster3d.h"

using namespace lidar_obstacle_detection;

template<typename PointT>
ClusterPts<PointT>::~ClusterPts() {}

template<typename PointT>
void ClusterPts<PointT>::clusterHelper(int ind, PtCdtr<PointT> cloud, std::vector<int> &cluster, KdTree *tree) {
    processed[ind] = true;      //标记第ind个点为已处理
    cluster.push_back(ind);     //将第ind个点加入集群中

    // 将与第ind个点附近距离小于等于distanceTol的点全部找出来
    std::vector<int> nearest_point = tree->search(cloud->points[ind], distanceTol);

    for (int nearest_id:nearest_point) {
        if (!processed[nearest_id]) {   // 近点中没被处理的
            clusterHelper(nearest_id, cloud, cluster, tree);    //继续找其附近点，并加入集群
        }
    }
}

// 欧式聚类
template<typename PointT>
std::vector<PtCdtr<PointT>> ClusterPts<PointT>::EuclidCluster(PtCdtr<PointT> cloud) {
    
    KdTree *tree = new KdTree;  //KDTree分割k维数据空间，快速进行最近邻搜索
    
    for (int ind = 0; ind < num_points; ind++) {
        tree->insert(cloud->points[ind], ind);          //将第ind个点插入树中，插入方式是交替比较xyz的大小,ind就是点的id
    }

    for (int ind = 0; ind < num_points; ind++) {
        if (processed[ind]) {       // 第ind个点已经被处理过了，证明该点已经属于某个聚类，直接逃过
            //ind++;  noted
            continue;
        }
        std::vector<int> cluster_ind;
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        clusterHelper(ind, cloud, cluster_ind, tree);       // 获取第ind个点该类的集群内所有点的id，保存在cluster_ind中

        int cluster_size = cluster_ind.size();
        //集群尺寸大于下阈值，则表明不是噪声; 小于上阈值，则表明不是多个碰在一起的物体被错误的归为同一类
        if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize) {
            //将对应id的点加入该聚类点云中
            for (int i = 0; i < cluster_size; i++) {
                cloudCluster->points.push_back(cloud->points[cluster_ind[i]]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            clusters.push_back(cloudCluster);
        }
    }
    return clusters;
}