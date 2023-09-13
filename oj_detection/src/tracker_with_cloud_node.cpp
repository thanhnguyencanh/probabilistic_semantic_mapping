#include "tracker_with_cloud_node.h"

TrackerWithCloudNode::TrackerWithCloudNode() : _pnh("~")
{
    // parameter from launch file
    _pnh.param<std::string>("camera_info_topic", _camera_info_topic, "camera_info");
    _pnh.param<std::string>("point_topic", _point_topic, "points_raw");
    _pnh.param<std::string>("detection2d_topic", _detection2d_topic, "detection2d_result");
    _pnh.param<std::string>("detection3d_topic", _detection3d_topic, "detection3d_result");
    _pnh.param<float>("cluster_tolerance", _cluster_tolerance, 0.5);
    _pnh.param<int>("min_cluster_size", _min_cluster_size, 100);
    _pnh.param<int>("max_cluster_size", _max_cluster_size, 25000);

    // subscriber topic
    _camera_info_sub.subscribe(_nh, _camera_info_topic, 1);
    _point_sub.subscribe(_nh, _point_topic, 1);
    _detection2d_sub.subscribe(_nh, _detection2d_topic, 1);
    
    // Publisher topic
    _detection_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("detection_cloud", 1);
    _projection_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("projection_cloud", 1);
    _old_projection_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("old_projection_cloud", 1);
    _detection3d_pub = _nh.advertise<vision_msgs::Detection3DArray>(_detection3d_topic, 1);
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("detection_marker", 1);
    _label_pub = _nh.advertise<visualization_msgs::MarkerArray>("detection_label", 1);

    _sensor_fusion_sync = boost::make_shared<message_filters::Synchronizer<_sensor_fusion_sync_subs>>(10);
    _sensor_fusion_sync->connectInput(_camera_info_sub, _point_sub, _detection2d_sub);
    _sensor_fusion_sync->registerCallback(boost::bind(&TrackerWithCloudNode::syncCallback, this, _1, _2, _3));

    _tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));
}

void TrackerWithCloudNode::syncCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                                        const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                        const vision_msgs::Detection2DArrayConstPtr& detections2d_msg)
{
    // Initilalize variable
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    vision_msgs::Detection3DArray detections3d_msg;
    sensor_msgs::PointCloud2 detection_cloud_msg;
    sensor_msgs::PointCloud2 projection_cloud_msg;
    sensor_msgs::PointCloud2 old_project_cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> old_projection_cloud;
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::MarkerArray label_array_msg;

    // Camera Information
    _cam_model.fromCameraInfo(camera_info_msg);

    // Transformation from message to cloud
    transformed_cloud = msg2TransformedCloud(cloud_msg);

    // Start the timer
    double begin = ros::Time::now().toSec();

    std::tie(detections3d_msg, detection_cloud_msg, projection_cloud_msg) = projectCloud(transformed_cloud, detections2d_msg, cloud_msg->header);

    // End the timer
    double end = ros::Time::now().toSec();

    // Calculate the processing time
    double processing_time = end-begin;

    // Print the processing time
    ROS_INFO("Processing time Poindcloud processing: %f milliseconds", processing_time);
    for (const auto& point : transformed_cloud.points)
    {
        if (point.y <0.2 && point.y > -4)
        {
            auto point_project = point;
            point_project.y = 0.2;
            old_projection_cloud.push_back(point_project);
        }       
        
    }
    old_projection_cloud = cloud2TransformedCloud(old_projection_cloud, cloud_msg->header);
    pcl::toROSMsg(old_projection_cloud, old_project_cloud_msg);
    old_project_cloud_msg.header = cloud_msg->header;

    marker_array_msg = createMarkerArray(detections3d_msg);
    label_array_msg = createLabelArray(detections3d_msg);

    // Publish data
    _detection3d_pub.publish(detections3d_msg);
    _detection_cloud_pub.publish(detection_cloud_msg);
    _projection_cloud_pub.publish(projection_cloud_msg);
    _old_projection_cloud_pub.publish(old_project_cloud_msg);
    _marker_pub.publish(marker_array_msg);
    _label_pub.publish(label_array_msg);
}

// Convert from sensor message data to pcl data
pcl::PointCloud<pcl::PointXYZ> 
    TrackerWithCloudNode::msg2TransformedCloud(const sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    geometry_msgs::TransformStamped tf;

    try
    {
        tf = _tf_buffer->lookupTransform(_cam_model.tfFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp);
        pcl_ros::transformPointCloud(_cam_model.tfFrame(), tf.transform, *cloud_msg, transformed_cloud_msg);
        pcl::fromROSMsg(transformed_cloud_msg, transformed_cloud);
    }
    catch(tf2::TransformException& e)
    {
        ROS_WARN("%s", e.what());
    }
    
    return transformed_cloud;
}

// Extract pointcloud from detection results 
std::tuple<vision_msgs::Detection3DArray, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    TrackerWithCloudNode::projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                       const vision_msgs::Detection2DArrayConstPtr detections2d_msg,
                                       const std_msgs::Header header)
{
    pcl::PointCloud<pcl::PointXYZ> detection_cloud_raw;
    pcl::PointCloud<pcl::PointXYZ> projection_cloud_raw;
    pcl::PointCloud<pcl::PointXYZ> detection_cloud;
    pcl::PointCloud<pcl::PointXYZ> projection_cloud;
    pcl::PointCloud<pcl::PointXYZ> closest_detection_cloud;
    pcl::PointCloud<pcl::PointXYZ> combine_detection_cloud;
    pcl::PointCloud<pcl::PointXYZ> combine_projection_cloud;

    vision_msgs::Detection3DArray detections3d_msg;
    sensor_msgs::PointCloud2 combine_detection_cloud_msg;
    sensor_msgs::PointCloud2 combine_projection_cloud_msg;

    detections3d_msg.header = header;

    for (size_t i = 0; i < detections2d_msg->detections.size(); i++)
    {
        for (const auto& point : cloud.points)
        {
            cv::Point3d pt_cv(point.x, point.y, point.z);
            cv::Point2d uv = _cam_model.project3dToPixel(pt_cv);
            
            // Extract point in bounding box
            if (point.z > 0 && uv.x > 0 && point.y < 0.2 &&
                uv.x >= detections2d_msg->detections[i].bbox.center.x - detections2d_msg->detections[i].bbox.size_x / 2 &&
                uv.x <= detections2d_msg->detections[i].bbox.center.x + detections2d_msg->detections[i].bbox.size_x / 2 &&
                uv.y >= detections2d_msg->detections[i].bbox.center.y - detections2d_msg->detections[i].bbox.size_y / 2 &&
                uv.y <= detections2d_msg->detections[i].bbox.center.y + detections2d_msg->detections[i].bbox.size_y / 2)
            {
                detection_cloud_raw.push_back(point);
                auto point_project = point;
                point_project.y = 0.2;
                projection_cloud_raw.push_back(point_project);
            }
        }
        // Transformation coordinate from detection_cloud_raw to original cloud (if use lidar + camera -> can calibration)
        detection_cloud = cloud2TransformedCloud(detection_cloud_raw, header);
        projection_cloud = cloud2TransformedCloud(projection_cloud_raw, header);

        if (!detection_cloud.points.empty())
        {
            // Cluster point cloud
            closest_detection_cloud = euclideanClusterExtraction(detection_cloud,  detections2d_msg->detections[i].results);

            createBoundingBox(detections3d_msg, closest_detection_cloud, detections2d_msg->detections[i].results);

            // Add each class to main point set
            combine_detection_cloud.insert(combine_detection_cloud.end(), closest_detection_cloud.begin(),
                                     closest_detection_cloud.end());
            detection_cloud_raw.points.clear();

            combine_projection_cloud.insert(combine_projection_cloud.end(), projection_cloud.begin(),
                                     projection_cloud.end());
            projection_cloud_raw.points.clear();
        }
    }

    pcl::toROSMsg(combine_detection_cloud, combine_detection_cloud_msg);
    combine_detection_cloud_msg.header = header;

    pcl::toROSMsg(combine_projection_cloud, combine_projection_cloud_msg);
    combine_projection_cloud_msg.header = header;
    return std::forward_as_tuple(detections3d_msg, combine_detection_cloud_msg, combine_projection_cloud_msg);
}

// Transformation of cloud
pcl::PointCloud<pcl::PointXYZ> 
    TrackerWithCloudNode::cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                 const std_msgs::Header header)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    geometry_msgs::TransformStamped tf;

    try
    {
        tf = _tf_buffer->lookupTransform(header.frame_id, _cam_model.tfFrame(), header.stamp);
        pcl_ros::transformPointCloud(cloud, transformed_cloud, tf.transform);
    }
    catch(tf2::TransformException& e)
    {
        ROS_WARN("%s", e.what());
    }
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ> 
    TrackerWithCloudNode::euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ> cloud,
                                                     const std::vector<vision_msgs::ObjectHypothesisWithPose> detections_results)
{
    pcl::PointCloud<pcl::PointXYZ> closest_cluster;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    float min_distance = std::numeric_limits<float>::max();
    tree->setInputCloud(cloud.makeShared());
    ec.setInputCloud(cloud.makeShared());
    ec.setClusterTolerance(_cluster_tolerance);
    ec.setMinClusterSize(_min_cluster_size);
    ec.setMaxClusterSize(_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    // ROS_INFO_STREAM("\n class"+ std::to_string(detections_results[0].id) +", cluster" + std::to_string(cluster_indices.size()));

    int biggest_cluster_size = 0;
    int biggest_cluster_index = 0;

    if (cluster_indices.size() == 0)
    {
        return closest_cluster;
    }
    
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        auto id = cluster_indices.at(i).indices;

        if (id.size() > biggest_cluster_size)
        {
            biggest_cluster_size = id.size();
            biggest_cluster_index = i;
        }
        
    }

    auto id = cluster_indices.at(biggest_cluster_index);
    for (std::vector<int>::const_iterator pit = id.indices.begin(); pit != id.indices.end(); ++pit)
    {
        closest_cluster.points.push_back(cloud.points[*pit]);
    }
    // closest_cluster.points.push_back(cloud.points[id]);
    
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
    //     Eigen::Vector4f centroid;

    //     pit = it->

        // for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        // {
            // cloud_cluster.points.push_back(cloud.points[*pit]);
        //     cloud_cluster.points.push_back(cloud.points[*pit]);
        // }

        // pcl::compute3DCentroid(cloud_cluster, centroid);
        // float distance = centroid.norm();
        // if (distance < min_distance)
        // {
        //     min_distance = distance;
        //     closest_cluster = cloud_cluster;
        // }
    // }

    return closest_cluster;
}

void TrackerWithCloudNode::createBoundingBox(vision_msgs::Detection3DArray& detections3d_msg, 
                                           const pcl::PointCloud<pcl::PointXYZ> cloud,
                                           const std::vector<vision_msgs::ObjectHypothesisWithPose> detections_results)
{
    vision_msgs::Detection3D detection3d;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointXYZ min_pt, max_pt;
    Eigen::Vector4f centroid;
    Eigen::Vector4f bbox_center;
    Eigen::Vector4f transformed_bbox_center;
    Eigen::Affine3f transform;
    pcl::compute3DCentroid(cloud, centroid);

    // Caculate rotation angle
    double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
    transform = Eigen::Affine3f::Identity();
    // Around Z axis
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(cloud, transformed_cloud, transform);

    pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);

    // Caclulate bounding of point cloud
    transformed_bbox_center =
        Eigen::Vector4f((min_pt.x + max_pt.x) / 2, (min_pt.y + max_pt.y) / 2, (min_pt.z + max_pt.z) / 2, 1);

    bbox_center = transform.inverse() * transformed_bbox_center;

    detection3d.bbox.center.position.x = bbox_center[0];
    detection3d.bbox.center.position.y = bbox_center[1];
    detection3d.bbox.center.position.z = bbox_center[2];

    Eigen::Quaternionf q(transform.inverse().rotation());

    detection3d.bbox.center.orientation.x = q.x();
    detection3d.bbox.center.orientation.y = q.y();
    detection3d.bbox.center.orientation.z = q.z();
    detection3d.bbox.center.orientation.w = q.w();

    detection3d.bbox.size.x = max_pt.x - min_pt.x;
    detection3d.bbox.size.y = max_pt.y - min_pt.y;
    detection3d.bbox.size.z = max_pt.z - min_pt.z;
    detection3d.results = detections_results;

    detections3d_msg.detections.push_back(detection3d);
}

visualization_msgs::MarkerArray 
    TrackerWithCloudNode::createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg)
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < detections3d_msg.detections.size(); i++)
    {
        if (std::isfinite(detections3d_msg.detections[i].bbox.size.x) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.y) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.z))
        {
            visualization_msgs::Marker marker;
            marker.header = detections3d_msg.header;
            marker.ns = "detection";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detections3d_msg.detections[i].bbox.center;
            marker.scale.x = detections3d_msg.detections[i].bbox.size.x;
            marker.scale.y = detections3d_msg.detections[i].bbox.size.y;
            marker.scale.z = detections3d_msg.detections[i].bbox.size.z;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            // marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(marker);
        }
    }
    return marker_array;
}

visualization_msgs::MarkerArray 
    TrackerWithCloudNode::createLabelArray(const vision_msgs::Detection3DArray& detections3d_msg)
{
    visualization_msgs::MarkerArray marker_array;
    std::string className;

    for (size_t i = 0; i < detections3d_msg.detections.size(); i++)
    {
        if (detections3d_msg.detections[i].results[0]. id == 0)
        {
            className = "chair";
        }
        else if (detections3d_msg.detections[i].results[0]. id == 1)
        {
            className = "t-coffee";
        }
        else if (detections3d_msg.detections[i].results[0]. id == 2)
        {
            className = "t-conference";
        }
        else if (detections3d_msg.detections[i].results[0]. id == 3)
        {
            className = "desk";
        }
        else if (detections3d_msg.detections[i].results[0]. id == 4)
        {
            className = "sofa";
        }
        else
        {
            className = "whiteboard";
        }
        
        
        if (std::isfinite(detections3d_msg.detections[i].bbox.size.x) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.y) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.z))
        {
            visualization_msgs::Marker marker;
            marker.header = detections3d_msg.header;
            marker.ns = "detection";
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detections3d_msg.detections[i].bbox.center;
            marker.pose.position.y = detections3d_msg.detections[i].bbox.center.position.y - detections3d_msg.detections[i].bbox.size.y/2;
            marker.scale.x = 0;
            marker.scale.y = 0;
            marker.scale.z = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            // marker.lifetime = ros::Duration(0.5);
            marker.text = className;
            marker_array.markers.push_back(marker);
        }
    }
    return marker_array;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_with_cloud_node");
    TrackerWithCloudNode node;
    ros::spin();
}