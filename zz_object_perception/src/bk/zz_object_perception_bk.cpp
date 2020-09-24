#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/crop_box.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <tf/transform_listener.h>

///////////////////////////////////////
#include <Eigen/Core>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/gicp.h>

std::vector<ros::Publisher> pub_cluster_vec;
std::vector<ros::Publisher> pub_cluster_pose;


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

sensor_msgs::PointCloud2::Ptr original_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr voxel_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr passthrough_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr segmentation_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr cluster_cloud (new sensor_msgs::PointCloud2);

std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;

void callback(const PointCloud::ConstPtr& msg)
{

  time_t tstart, tend; 
  tstart = time(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>());
  PointCloudT::Ptr object_aligned (new PointCloudT);
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr object_normals(new pcl::PointCloud<pcl::Normal>);
  PointCloudT::Ptr scene (new PointCloudT);  
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


  pc2_clusters.clear();

  std::string world_frame="xtion_optical_frame";

  //Convertir point cloud PCL->ROS

  pcl::toROSMsg(*msg, *original_cloud);
  original_cloud->header.frame_id = world_frame;
  original_cloud->header.stamp = ros::Time::now();

  //voxel grid filter
  float leaf = 0.005f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*msg));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud_ptr);
  voxel_filter.setLeafSize(float(leaf), float(leaf), float(leaf)); //In Meters
  voxel_filter.filter(*cloud_voxel_filtered);
  ROS_INFO_STREAM("DownSampled scene cloud with " << cloud_voxel_filtered->size() << "points");

  pcl::toROSMsg(*cloud_voxel_filtered, *voxel_cloud);
  voxel_cloud->header.frame_id = world_frame;
  voxel_cloud->header.stamp = ros::Time::now();
  ROS_INFO_STREAM("DownSampled scene cloud with " << cloud_voxel_filtered->size() << "points");

  //passthrough filter
  pcl::PointCloud<pcl::PointXYZ> yf_cloud, zf_cloud;

  /*  //filter in x
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-0.5,0.0);
  pass_x.filter(xf_cloud);
  */
  
  //filter in y
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud_voxel_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-0.4,0.5);
  pass_y.filter(yf_cloud);

    //filter in z
  pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.0,1.0);
  pass_z.filter(zf_cloud);

  ROS_INFO_STREAM("DownSampled Passthrough filter with "<<zf_cloud.size()<<" points");
  pcl::toROSMsg(zf_cloud, *passthrough_cloud);
  passthrough_cloud->header.frame_id = world_frame;
  passthrough_cloud->header.stamp = ros::Time::now();

  //plane segmentation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

//Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(500);
  seg.setDistanceThreshold(0.01);

//Segment the largest planar component from the cropped cloud
  seg.setInputCloud(cropped_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
  }

//Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);

//Get the points associated with the planar surface
  extract.filter(*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size() << "data points.");

//Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);


//euclidean cluster extraction 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  *cloud_filtered = *cloud_f;
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  //segmentation
  pcl::toROSMsg(*cloud_f, *segmentation_cloud);
  segmentation_cloud->header.frame_id = world_frame;
  segmentation_cloud->header.stamp = ros::Time::now();
  ros::NodeHandle nh;

  //Create a publisher for each cluster
  for (int i=0; i<cluster_indices.size(); ++i) {
    std::string topicName = "/cluster_" + boost::lexical_cast<std::string>(i);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);
    pub_cluster_vec.push_back(pub);

    ros::Publisher pub_po = nh.advertise<geometry_msgs::PointStamped> ("/cluster_" + boost::lexical_cast<std::string>(i) + "/pose", 1);
    pub_cluster_pose.push_back(pub_po);
  }

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
      clusters.push_back(cloud_cluster);

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster, centroid);
      geometry_msgs::PointStamped cluster_pose_temp;
      cluster_pose_temp.header.frame_id = "xtion_optical_frame";
      cluster_pose_temp.header.stamp = ros::Time::now();
      cluster_pose_temp.point.x = centroid[0];
      cluster_pose_temp.point.y = centroid[1];
      cluster_pose_temp.point.z = centroid[2];

      std::cout << "centroid point is  " << cluster_pose_temp << " .\n";
      pub_cluster_pose[j].publish(cluster_pose_temp);

    
      sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
      pc2_clusters.push_back(tempROSMsg);
      tempROSMsg->header.frame_id = world_frame;
      tempROSMsg-> header.stamp = ros::Time::now();
      pub_cluster_vec[j].publish(tempROSMsg);
      ++j;
  }



//statical outlier removal
/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *cluster_cloud_ptr = *cloud_f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ>sor;
  sor.setInputCloud(cluster_cloud_ptr);
  sor.setMeanK(100);
  sor.setStddevMulThresh(1.0);
  sor.filter(*sor_cloud_filtered);

  ROS_INFO_STREAM("Removal statical outlier: "<<sor_cloud_filtered->points.size()<<"data points.");

  //outlier
  pcl::toROSMsg(*sor_cloud_filtered, *outlier_cloud);
  outlier_cloud->header.frame_id = world_frame;
  outlier_cloud->header.stamp = ros::Time::now();

  //broadcast transform
  for(int i=0; i<j; i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_vec= clusters.at(i);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_vec, centroid);
    Eigen::Vector3f vec3 = centroid.head<3>();

    //Broadcast transform
    static tf::TransformBroadcaster br;
    tf::Transform part_transform;

    part_transform.setOrigin(tf::Vector3(vec3(0),vec3(1),vec3(2)));
    tf::Quaternion q;
    q.setRPY(0 ,0, 0);
    part_transform.setRotation(q);

    std::stringstream ss;
    ss << "cluster_" << i;

    ROS_INFO_STREAM(vec3);

    br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, ss.str()));
    
  }*/


//2. Estimate normals for the scene
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nest;
  nest.setInputCloud(cloud_filtered);
  nest.setKSearch(5);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  nest.setSearchMethod(kdtree);
  nest.compute(*scene_normals);

//Estimate normals for the object
  nest.setInputCloud(object);
  nest.compute(*object_normals);

//Estimate features
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object_normals);
  fest.compute(*object_features);
  fest.setInputCloud(cloud_filtered);
  fest.setInputNormals(scene_normals);
  fest.compute(*scene_features);

  ROS_INFO_STREAM("Scene descriptors: " << scene_features->points.size() << " points");
  ROS_INFO_STREAM("Object descriptors: " << object_features->points.size() << " points");*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



//3. Perform alignment
///////////////////////////////////////////////////////////////////////////////////////////////////////////////  
 /* pcl::PointCloud<PointType>::Ptr align_cloud(new pcl::PointCloud<PointType>);

  pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac;
  sac.setMinSampleDistance(0.05f);
  sac.setMaxCorrespondenceDistance(0.01f*0.01f);//0.005f
  sac.setMaximumIterations(500);//5000------300ok
  sac.setRANSACIterations(500);//500
  sac.setNumberOfSamples(5);//5
  sac.setCorrespondenceRandomness(25);//25
  sac.setInputSource(object);
  sac.setSourceFeatures(object_features);
  sac.setInputTarget(cloud_filtered);
  sac.setTargetFeatures(scene_features);
  sac.align(*align_cloud);

  //aligned cloud
  pcl::toROSMsg(*align_cloud, *sac_pub);
  sac_pub->header.frame_id = world_frame;
  sac_pub->header.stamp = ros::Time::now();

  Eigen::Matrix4f ia_trans = sac.getFinalTransformation();

  std::cout << ia_trans << std::endl;*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //4. Refinement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /*pcl::console::print_highlight ("ICP refinement...\n");

  pcl::PointCloud<PointType>::Ptr icp_cloud(new pcl::PointCloud<PointType>);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  
  typedef pcl::registration::TransformationEstimationLM <PointType, PointType> te;
  boost::shared_ptr<te> teLM (new te);
  icp.setTransformationEstimation (teLM);
  icp.setInputSource(align_cloud);
  icp.setInputTarget(cloud_filtered);
  icp.setMaxCorrespondenceDistance(0.0005f);//0.01
  icp.setMaximumIterations(1000);//500
  icp.setTransformationEpsilon(1e-4);//1e-4
  icp.setRANSACOutlierRejectionThreshold(0.005);//0.01
  icp.align(*icp_cloud);

  ROS_INFO_STREAM("Score: " << icp.getFitnessScore());

  Eigen::Matrix4f transformation = icp.getFinalTransformation();*/
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // get position and orientation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*transformation = transformation * ia_trans;

  std::cout << transformation << std::endl;

  double roll, pitch, yaw;
  tf::Matrix3x3 RPY;

  RPY.setValue(transformation(0,0), transformation(0,1), transformation(0,2), 
               transformation(1,0), transformation(1,1), transformation(1,2), 
               transformation(2,0), transformation(2,1), transformation(2,2));
  RPY.getRPY(roll, pitch, yaw);

    static tf::TransformBroadcaster br;
    tf::Transform part_transform;

    part_transform.setOrigin(tf::Vector3(transformation(0,3), transformation(1,3), transformation(2,3)));
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    part_transform.setRotation(q);

        ROS_INFO_STREAM("\nX: " << transformation(0,3) << 
                  "\nY: " << transformation(1,3) << 
                  "\nZ: " << transformation(2,3) <<
                  "\nRoll: " << roll <<
                  "\nPitch: " << pitch <<
                  "\nYaw: " << yaw);

    br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "object_aligned"));
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    tend = time(0); 
    std::cout << "Alignment finished. It took "<< difftime(tend, tstart) <<" second(s)."<< std::endl;



  //aligned cloud
  pcl::toROSMsg(*icp_cloud, *icp_pub);
  icp_pub->header.frame_id = world_frame;
  icp_pub->header.stamp = ros::Time::now();
*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  //Subscribe to the kinect point cloud topic
  ros::Subscriber sub = nh.subscribe<PointCloud>("/xtion/depth_registered/points", 1, callback);

  //Create a ROS publisher for the output point cloud
  ros::Publisher pub_original = nh.advertise<PointCloud>("original_cloud", 1);
  //voxel publisher
  ros::Publisher pub_voxel = nh.advertise<PointCloud>("voxel_filter", 1);
  //passthrough publisher
  ros::Publisher pub_passthrough = nh.advertise<PointCloud>("passthrough_filter", 1);
  //segmentation publisher
  ros::Publisher pub_segmentation = nh.advertise<PointCloud>("segmentation_filter", 1);
  //outlier removal publisher
  //ros::Publisher pub_outlier = nh.advertise<PointCloud>("outlier_removal", 1);
  //alignment publisher
  //ros::Publisher pub_sac = nh.advertise<PointCloud>("sac_cloud", 1);
  //alignment publisher
  //ros::Publisher pub_icp = nh.advertise<PointCloud>("aligned_cloud", 1);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //Publish the original point cloud
    pub_original.publish(original_cloud);

    //Publish the processed point cloud by the voxel filter
    pub_voxel.publish(voxel_cloud);

    //Publish the processed point cloud by the passthrough filter
    pub_passthrough.publish(passthrough_cloud);
    
    //Publish the processed point cloud by the plane segmentation
    pub_segmentation.publish(segmentation_cloud);

    //Publish the processed point cloud by the statical outlier removal
    //pub_outlier.publish(outlier_cloud);

    //pub_sac.publish(sac_pub);

    //pub_icp.publish(icp_pub);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

return 0;
}
