#include <ros/ros.h>                                    // Core ROS functionality (node, pub/sub, logs)
#include <sensor_msgs/PointCloud2.h>                    // ROS message type for point clouds
#include <pcl_conversions/pcl_conversions.h>            // Conversion between ROS and PCL
#include <pcl/point_cloud.h>                            // PCL point cloud container
#include <pcl/point_types.h>                            // Common PCL point types (PointXYZ, etc.)
#include <pcl/filters/voxel_grid.h>                     // Exact voxel grid filter (not used here)
#include <pcl/filters/approximate_voxel_grid.h>         // Faster approximate voxel grid filter
#include <sensor_msgs/point_cloud2_iterator.h>          // Iterators for reading/writing PointCloud2 fields

ros::Publisher pub;                                     // Global publisher for the output point cloud


// ============================================================================
// Callback function executed each time a new point cloud arrives on /detected_wall
// ============================================================================
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 1) Copy the incoming PointCloud2 message
    //    This prevents modifying the original input buffer.
    sensor_msgs::PointCloud2 projected = *input;

    // 2) In-place projection: set all Z values to 0.0
    //    This effectively projects the point cloud onto a flat ground plane.
    for (sensor_msgs::PointCloud2Iterator<float> it(projected, "z"); it != it.end(); ++it) {
        *it = 0.0f;                                     // Overwrite Z field
    }

    // 3) Convert the modified PointCloud2 (ROS) → PCL format
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(projected, cloud);                  // Convert ROS → PCL structure

    // 4) Downsample the projected cloud using an ApproximateVoxelGrid filter
    //    This reduces point count while maintaining overall structure.
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;       // Faster voxel grid filter

    sor.setInputCloud(cloud.makeShared());              // Input cloud
    sor.setLeafSize(0.4f, 0.4f, 0.4f);                  // Voxel size (40 cm cubes)
    sor.filter(cloud_filtered);                         // Apply filtering

    // 5) Convert filtered cloud back to ROS message and publish it
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);              // PCL → ROS conversion

    output.header = projected.header;                   // Preserve original frame + timestamp
    pub.publish(output);                                // Publish on /compressed_wall
}


// ============================================================================
// Main function: initializes the node, subscriber, publisher, and spinning
// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_compress_node");        // Initialize ROS and name the node
    ros::NodeHandle nh;                                 // Node handle for communication

    // Subscriber to /detected_wall topic
    // - Queue size: 1 (only latest frame needed)
    // - tcpNoDelay: reduces communication latency
    ros::Subscriber sub = nh.subscribe(
        "/detected_wall",
        1,
        cloudCallback,
        ros::TransportHints().tcpNoDelay(true)
    );

    // Publisher for the processed/filtered point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/compressed_wall", 1);

    // Use a multi-threaded spinner for higher throughput (good for point cloud processing)
    ros::AsyncSpinner spinner(4);                       // Use 4 threads
    spinner.start();                                    // Begin handling callbacks concurrently

    ros::waitForShutdown();                             // Keep the node alive until shutdown
    return 0;
}
