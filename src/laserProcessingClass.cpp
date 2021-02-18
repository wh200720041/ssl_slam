// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    //coordinate transform
    for (int i = 0; i < (int) pc_in->points.size(); i++){
        double new_x = pc_in->points[i].z;
        double new_y = -pc_in->points[i].x;
        double new_z = -pc_in->points[i].y;
        pc_in->points[i].x = new_x;
        pc_in->points[i].y = new_y;
        pc_in->points[i].z = new_z;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserCloudScans;

    double last_angle = atan2(pc_in->points[0].z,pc_in->points[0].y) * 180 / M_PI;
    int count =0;
    int point_size = pc_in->points.size()-1;
    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        if(pc_in->points[i].z<-1.0||pc_in->points[i].z>1.0||pc_in->points[i].x<0.5 )
            continue;

        int scanID=0;
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y + pc_in->points[i].z * pc_in->points[i].z);
        double angle = atan2(pc_in->points[i].x,pc_in->points[i].z) * 180 / M_PI;
        count++;

        if(fabs(angle - last_angle)>0.05){
            
            if(count>100){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
                for(int k=0;k<count;k++){
                    pc_temp->push_back(pc_in->points[i-count+k+1]);
                    //pc_in->points[i-count+k+1].intensity=1.0;
                }
                laserCloudScans.push_back(pc_temp);
            }
            count =0;
            last_angle = angle;
        }

    }

    ROS_WARN_ONCE("total points array %d", laserCloudScans.size());

    for(int i = 0; i < laserCloudScans.size(); i++){
        
        std::vector<Double2d> cloudCurvature; 
        int total_points = laserCloudScans[i]->points.size()-10;
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }

        featureExtractionFromSector(laserCloudScans[i],cloudCurvature, pc_out_edge, pc_out_surf);
            
    
    }

}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf){

    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
    { 
        return a.value < b.value; 
    });


    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if (largestPickedNum <= 10){
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }
    
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
    


}
LaserProcessingClass::LaserProcessingClass(){
    
}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
    layer = layer_in;
    time = time_in;
};
