// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _OCTO_MAPPING_H_
#define _OCTO_MAPPING_H_

//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//c++ lib
#include <string>
#include <math.h>
#include <vector>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#define MIN_MAP_UPDATE_DISPLACEMENT 0.5 //only update the global map when the displacement is greater than this threshold

class OctoMappingClass 
{

    public:
    	OctoMappingClass();
		void init(double resolution);
		void updateLocalMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, const Eigen::Isometry3d& pose_current);
		void updateGlobalMap(void);
		octomap::ColorOcTree* getMap(void);
		

	private:

		octomap::ColorOcTree* map;
		octomap::ColorOcTree* local_map;
		Eigen::Isometry3d last_pose;

};


#endif // _OCTO_MAPPING_H_

