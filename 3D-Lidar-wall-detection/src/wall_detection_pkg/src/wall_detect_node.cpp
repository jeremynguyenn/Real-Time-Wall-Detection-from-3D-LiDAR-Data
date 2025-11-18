#include <ros/ros.h>                                     // Core ROS functionality
#include <sensor_msgs/PointCloud2.h>                     // ROS message type for point clouds
#include <pcl_conversions/pcl_conversions.h>             // ROS ↔ PCL message conversion
#include <pcl/point_types.h>                             // PCL point type definitions
#include <pcl/point_cloud.h>                             // PCL point cloud container
#include <pcl/filters/voxel_grid.h>                      // Exact voxel downsampling
#include <pcl/filters/passthrough.h>                     // PassThrough filter (ROI filtering)
#include <pcl/filters/extract_indices.h>                 // Extract inliers/outliers from clouds
#include <pcl/filters/statistical_outlier_removal.h>     // Noise removal filter
#include <pcl/ModelCoefficients.h>                       // Used for storing RANSAC plane model
#include <pcl/segmentation/sac_segmentation.h>           // RANSAC-based segmentation
#include <pcl/filters/approximate_voxel_grid.h>          // Fast voxel downsampling filter


// ============================================================================
// WallDetector class: performs filtering + plane segmentation to detect vertical walls
// ============================================================================
class WallDetector {
public:
    WallDetector() {
        // Subscriber: receives raw Velodyne LiDAR point clouds
        sub_ = nh_.subscribe(
            "/velodyne_points",             // Input LiDAR topic
            1,                              // Queue size
            &WallDetector::pointCloudCallback,
            this,
            ros::TransportHints().tcpNoDelay(true)   // Reduce latency
        );

        // Publisher: outputs extracted wall point clouds
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detected_wall", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;


    // ============================================================================
    // Callback: executed every time a new LiDAR point cloud arrives
    // ============================================================================
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {

        // 1) Convert ROS PointCloud2 → PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 2) ROI filter 1: Keep points where 0 < x < 6
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.0f, 6.0f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_x(new pcl::PointCloud<pcl::PointXYZ>);
        pass_x.filter(*cloud_roi_x);

        // 3) ROI filter 2: Keep points where -3 < y < 3
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_roi_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-3.0f, 3.0f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_y(new pcl::PointCloud<pcl::PointXYZ>);
        pass_y.filter(*cloud_roi_y);

        // 4) ROI filter 3: Keep points where -1 < z < 2
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud_roi_y);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-1.0f, 2.0f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        pass_z.filter(*cloud_roi);

        // 5) Downsample using ApproximateVoxelGrid (leaf size = 5 cm)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_roi);
        voxel.setLeafSize(0.05f, 0.05f, 0.05f);          // 5 cm voxel resolution
        voxel.filter(*cloud_down);

        // 6) Additional Z-filter: remove floor and ceiling (-1 < z < 1)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_down);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.0, 1.0);
        pass.filter(*cloud_filtered);

        // 7) Setup RANSAC plane segmentation to detect wall-like planes
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);              // Improve accuracy
        seg.setModelType(pcl::SACMODEL_PLANE);          // Fit planar surfaces
        seg.setMethodType(pcl::SAC_RANSAC);             // RANSAC algorithm
        seg.setMaxIterations(60);                       // Max iterations per plane
        seg.setDistanceThreshold(0.02);                 // 2 cm threshold for inliers

        pcl::ExtractIndices<pcl::PointXYZ> extract;      // Extract plane inliers/outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Minimum number of points required to consider a plane valid
        constexpr size_t MIN_WALL_POINTS = 50;

        // Attempt to extract up to 10 walls or planes
        for (int i = 0; i < 10; ++i) {
            if (cloud_filtered->points.empty()) break;

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

            // 7.1) Perform RANSAC plane segmentation
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coeff);
            if (inliers->indices.empty()) break;

            // Compute plane normal vector
            Eigen::Vector3f plane_normal(coeff->values[0],
                                         coeff->values[1],
                                         coeff->values[2]);
            plane_normal.normalize();

            float nz = plane_normal[2];                 // Z component of the normal

            // Threshold for near-vertical planes (nz close to 0)
            const float cos_80 = 0.1736f;
            bool isWall = (std::fabs(nz) < cos_80);

            if (isWall) {
                // 7.2) Extract plane inliers (wall points)
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);
                pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>);
                extract.filter(*plane_points);

                // Reject small noisy planes
                if (plane_points->points.size() < MIN_WALL_POINTS) {
                    extract.setNegative(true);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
                    extract.filter(*tmp);
                    cloud_filtered.swap(tmp);
                    continue;
                }

                // Add valid wall points to final output
                wall_cloud->points.insert(
                    wall_cloud->points.end(),
                    plane_points->points.begin(),
                    plane_points->points.end()
                );
            }

            // 7.4) Remove the extracted plane from cloud and continue searching
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*remaining);
            cloud_filtered.swap(remaining);
        }

        // 8) Convert final wall point cloud back to ROS message and publish it
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*wall_cloud, output_msg);
        output_msg.header = cloud_msg->header;         // Preserve timestamp + frame ID
        pub_.publish(output_msg);                      // Publish to /detected_wall
    }
};


// ============================================================================
// Main entry point
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_detect_node");        // Initialize ROS node
    WallDetector wd;                                  // Create WallDetector instance
    ROS_INFO("wall_detect_node started.");            // Debug message
    ros::spin();                                      // Keep node running
    return 0;
}
