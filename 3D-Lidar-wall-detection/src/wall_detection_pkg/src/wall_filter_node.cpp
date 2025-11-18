#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>  // for pcl::fromROSMsg and pcl::toROSMsg
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>           // Voxel downsampling
#include <pcl/filters/passthrough.h>          // ROI filtering
#include <pcl/filters/statistical_outlier_removal.h>  // Noise removal
#include <pcl/sample_consensus/model_types.h>         // RANSAC model types
#include <pcl/sample_consensus/method_types.h>        // RANSAC methods
#include <pcl/segmentation/sac_segmentation.h>        // RANSAC segmentation
#include <pcl/filters/extract_indices.h>              // Extract inliers/outliers

ros::Publisher wall_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {

    // 1) Convert incoming ROS PointCloud2 message → PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_msg, *cloud);

    // 2) Downsample the point cloud using VoxelGrid (optional)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.02f, 0.02f, 0.02f);  // 2 cm voxel resolution
    voxel.filter(*cloud_down);

    // 3) Configure RANSAC plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);              // Improve plane fitting accuracy
    seg.setModelType(pcl::SACMODEL_PLANE);          // Detect planar surfaces
    seg.setMethodType(pcl::SAC_RANSAC);             // Use RANSAC algorithm
    seg.setMaxIterations(200);                      // Max iterations for RANSAC
    seg.setDistanceThreshold(0.05);                 // Points within 5 cm are considered on the plane

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Run plane segmentation on the downsampled cloud
    seg.setInputCloud(cloud_down);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No plane found by RANSAC.");
        return;
    }

    // 4) Check plane normal to remove horizontal planes (floor/ceiling)
    Eigen::Vector3f normal(coefficients->values[0],
                           coefficients->values[1],
                           coefficients->values[2]);
    normal.normalize();

    // Compute angle between plane normal and Z-axis
    double cos_angle = std::fabs(normal.dot(Eigen::Vector3f::UnitZ()));
    double cos15 = std::cos(15 * M_PI / 180.0);  // ≈ 0.9659 (within 15° of horizontal)

    if (cos_angle >= cos15) {
        // Plane is horizontal → remove and retry segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rem(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_down);
        extract.setIndices(inliers);
        extract.setNegative(true);                // Remove horizontal plane
        extract.filter(*cloud_rem);

        if (!cloud_rem->empty()) {
            // Try RANSAC again on remaining points
            seg.setInputCloud(cloud_rem);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                ROS_WARN("After removing horizontal plane, no plane was found.");
                return;
            }

            // Recompute plane normal
            normal = Eigen::Vector3f(coefficients->values[0],
                                     coefficients->values[1],
                                     coefficients->values[2]);
            normal.normalize();
            cos_angle = std::fabs(normal.dot(Eigen::Vector3f::UnitZ()));
        }
    }

    // 5) Check if detected plane is vertical (wall)
    // cos(75°) ≈ 0.2588 → within 15° of vertical
    double cos75 = std::cos(75 * M_PI / 180.0);

    if (cos_angle < cos75) {
        // The plane is vertical → treat as a wall
    } else {
        ROS_WARN("Detected plane is NOT vertical → not a wall.");
    }

    // 6) Extract plane inliers (wall points) from the downsampled cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_down);    // Extract from downsampled cloud
    extract.setIndices(inliers);
    extract.setNegative(false);           // Keep only plane points
    extract.filter(*wall_cloud);

    // 7) Height filtering: keep points only between 0.5 m and 2.0 m
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(wall_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 2.0);       // Keep points between 0.5m and 2.0m height
    pass.filter(*wall_filtered);

    // 8) Statistical outlier removal (optional noise reduction)
    if (!wall_filtered->empty()) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(wall_filtered);
        sor.setMeanK(50);                // Number of neighbors to analyze
        sor.setStddevMulThresh(1.0);     // Reject points beyond 1σ
        sor.filter(*wall_filtered);
    }

    // 9) Publish result as ROS PointCloud2
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*wall_filtered, output_msg);
    output_msg.header = input_msg->header;     // Preserve timestamp & frame
    wall_pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_filter_node");
    ros::NodeHandle nh;

    // Subscriber for raw Velodyne point cloud
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pointCloudCallback);

    // Publisher for extracted wall points
    wall_pub = nh.advertise<sensor_msgs::PointCloud2>("/wall_points_filtered", 1);

    ROS_INFO("Wall filter node started: waiting for point cloud...");
    ros::spin();
    return 0;
}


// Notes:
// 3) Increasing setDistanceThreshold allows rough/uneven walls to still be detected.
// 5) Making the cosine threshold closer to 90° forces stricter vertical detection.
// 6) setInputCloud can take original or downsampled cloud depending on desired output.
