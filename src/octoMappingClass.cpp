// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "octoMappingClass.h"
#include <chrono>
//commment on probhit and miss
//miss<0.5, closer to 0.5 means less reduce in probability 
//hit closer to 1 means higher increase in probability
void OctoMappingClass::init(double resolution){
	//init map
	octomap::ColorOcTree* map_temp = new octomap::ColorOcTree(resolution);
	map = map_temp; 
	octomap::ColorOcTree* map_temp2 = new octomap::ColorOcTree(resolution);
	local_map = map_temp2; 

	local_map->setProbHit(0.85);
	local_map->setProbMiss(0.45);

	map->setProbHit(0.85);
	map->setProbMiss(0.45);
	last_pose = Eigen::Isometry3d::Identity();
	last_pose.translation().x() = -10;
}

void OctoMappingClass::updateGlobalMap(void){
	// Expand local_map so we search all nodes
	local_map->expand();

	// traverse nodes in local_map to add them to tree1
	for (octomap::ColorOcTree::leaf_iterator it = local_map->begin_leafs(); it != local_map->end_leafs(); ++it) {

	  // find if the current node maps a point in map1
	  octomap::point3d point = it.getCoordinate();
	  octomap::ColorOcTreeNode *node = map->search(point);
	  if (node != NULL) {
	    // Add the probability of local_map node to the found node
	    octomap::OcTreeKey nodeKey = map->coordToKey(point);
	    map->updateNode(nodeKey, it->getLogOdds());
	  } else {
	    // Create a new node and set the probability from local_map
	    octomap::ColorOcTreeNode *newNode = map->updateNode(point, true);
	    newNode->setLogOdds(it->getLogOdds());
	  }
	}

}

//update points to map 
void OctoMappingClass::updateLocalMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, const Eigen::Isometry3d& pose_current){

	//check for global map update
	double displacement = std::sqrt((last_pose.translation()-pose_current.translation()).transpose() * (last_pose.translation()-pose_current.translation()));
	if(displacement>MIN_MAP_UPDATE_DISPLACEMENT){

		//update to local map
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*pc_in, *transformed_temp, pose_current.cast<float>());

		octomap::Pointcloud cloud_octo;
		for(int i=0;i<(int)transformed_temp->points.size();i++){
			cloud_octo.push_back( transformed_temp->points[i].x, transformed_temp->points[i].y, transformed_temp->points[i].z );
		}
		double max_range = 10.0;
		local_map->insertPointCloud(cloud_octo,octomap::point3d(pose_current.translation().x(),pose_current.translation().y(),pose_current.translation().z()), max_range);


        updateGlobalMap();

		for (int i=0;i<pc_in->points.size();i++) {
		  
		  octomap::ColorOcTreeNode* node = map->search(transformed_temp->points[i].x,transformed_temp->points[i].y,transformed_temp->points[i].z);
           if (node)
                node->setColor(transformed_temp->points[i].r, transformed_temp->points[i].g, transformed_temp->points[i].b);

		}

		last_pose = pose_current;
		local_map->clear();
	} 

}

octomap::ColorOcTree* OctoMappingClass::getMap(void){
	return map;
}

OctoMappingClass::OctoMappingClass(){

}
