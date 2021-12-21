#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <cmath>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_exports.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
typedef pcl::PointXYZ PointT;

int main(int argc,char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    std::string cloudPath="../src/table_scene_lms400.pcd";
    pcl::io::loadPCDFile(cloudPath,*cloud);
    //pcl::io::loadPLYFile(argv[1],*cloud); //load ply format;

    //normal estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<PointT,pcl::Normal> ne;
    tree->setInputCloud(cloud);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);    

    int k = 20;
    int points_num = cloud->points.size();
    vector<int> k_nebor_index;
    vector<float> k_nebor_index_dis;
    vector<vector<int>> point_k_index;
    point_k_index.resize(points_num,k_nebor_index);
    for (size_t i = 0; i < points_num; i++)
    {
       if (tree->nearestKSearch(cloud->points[i],k,k_nebor_index,k_nebor_index_dis))
       {
          point_k_index[i].swap(k_nebor_index);
       }
       else
       {
           PCL_ERROR("WARNING:FALSE NEARERTKSEARCH");
       }
    }

    vector<pair<float,int>> vec_curvature;
    for (size_t i = 0; i < points_num; i++)
    {
        vec_curvature.push_back(make_pair(normals->points[i].curvature,i));
    }
    sort(vec_curvature.begin(),vec_curvature.end());

    //TODO:计算点周围20个点的曲率并根据曲率进行了排序，同时存储了kdtree搜索到的对应的点的索引

    float curvature_threshold = 0.1;
    float normal_threshold = cosf(10.0/180.0*M_PI);
    int seed_orginal = vec_curvature[0].second;
    int counter_0 = 0;
    int segment_laber(0);
    vector<int> segmen_num;
    vector<int> point_laber;
    point_laber.resize(points_num,-1);
    while(counter_0 < points_num)
    {   
        queue<int> seed;
        seed.push(seed_orginal);
        point_laber[seed_orginal] = segment_laber;
        int counter_1(1);
        while(!seed.empty())
        {
            int curr_seed = seed.front();
            seed.pop();
            int curr_nebor_index(0);
            while (curr_nebor_index<k)
            {
                bool is_a_seed = false;
                int cur_point_idx = point_k_index[curr_seed][curr_nebor_index]; //遍历当前点对应的K个点
                if (point_laber[cur_point_idx] != -1) //当前点已经被标记过，不参与后面计算
                {
                    curr_nebor_index++;
                    continue;
                }
                Eigen::Map<Eigen::Vector3f> vec_curr_point(static_cast<float*>(normals->points[curr_seed].normal));
                Eigen::Map<Eigen::Vector3f> vec_nebor_point (static_cast<float*>(normals->points[cur_point_idx].normal));
                float dot_normals = fabsf(vec_curr_point.dot(vec_nebor_point));
                if(dot_normals<normal_threshold)  // dot_normals越小，夹角越大，要求夹角小
                {
                    is_a_seed = false;
                }
                else if(normals->points[cur_point_idx].curvature>curvature_threshold)
                {
                    is_a_seed = false;
                }
                else
                {
                    is_a_seed = true;
                }                
                if (!is_a_seed)
                {
                    curr_nebor_index++;
                    continue;
                }
                point_laber[cur_point_idx] = segment_laber; //标记点
                counter_1++;   //已经标记的点个数
                if (is_a_seed)
                {
                    seed.push(cur_point_idx);
                }
                curr_nebor_index++;  
            } 
            std::cout<<"seed.size():"<<seed.size()<<std::endl;     
        } // 第一个种子点遍历完成
        segment_laber++; 
        counter_0 +=counter_1;
        segmen_num.push_back(counter_1);
        for (size_t i = 0; i < points_num; i++)
        {
            int index_curvature = vec_curvature[i].second;
            if(point_laber[index_curvature] == -1)
            {
                seed_orginal = index_curvature;
                break;
            }           
        }   //寻找下一个聚类的初始种子点
    }
    cout<<"seg_num:"<<segmen_num.size()<<endl;//每个类对应的点的个数
    //summary of segmentation results
    vector<pcl::PointIndices> cluster;
    pcl::PointIndices segment_points;
    int seg_num = segmen_num.size();
    cluster.resize(seg_num,segment_points);
    for(size_t i_seg = 0;i_seg<seg_num;i_seg++)
    {
        cluster[i_seg].indices.resize(segmen_num[i_seg],0);
    }
    //根据索引取点云中对应的点，归到对应的类中去
    vector<int> counter_2;
    counter_2.resize(seg_num,0);
    for(size_t i_point =0 ;i_point<points_num;i_point++)
    {
        int seg_idx = point_laber[i_point];
        int nebor_idx = counter_2[seg_idx];
        cluster[seg_idx].indices[nebor_idx] = i_point;
        counter_2[seg_idx] +=1;  //每个聚类点对应的点个数
    }

    //Remove outline points
    vector<pcl::PointIndices> clusters;
    int minNum = 100; 
    int maxNum = 100000;
    for(size_t i_cluster = 0;i_cluster<seg_num;i_cluster++)
    {
        if(cluster[i_cluster].indices.size() < maxNum && cluster[i_cluster].indices.size() > minNum)
        {
            clusters.push_back(cluster[i_cluster]);
        }
    }

    // visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RegionGrowing Viewer"));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
    srand(time(nullptr));
    vector<unsigned char> color;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        color.push_back(static_cast<unsigned char>(rand()%256));
        color.push_back(static_cast<unsigned char>(rand()%256));
        color.push_back(static_cast<unsigned char>(rand()%256));
    }
    int color_index(0);
    for(size_t i = 0;i<clusters.size();i++)
    {
        for(size_t j = 0; j<clusters[i].indices.size();j++)
        {
            pcl::PointXYZRGB n_points;
            n_points.x = cloud->points[clusters[i].indices[j]].x;
            n_points.y = cloud->points[clusters[i].indices[j]].y;
            n_points.z = cloud->points[clusters[i].indices[j]].z;
            n_points.r = color[3*color_index];
            n_points.g = color[3*color_index+1];
            n_points.b = color[3*color_index+2];
            cloud_color->push_back(n_points);
        }
        color_index++;
    }
    viewer->addPointCloud(cloud_color);
    viewer->spin();
    return 0;
}