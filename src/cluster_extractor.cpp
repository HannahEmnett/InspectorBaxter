#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <inspector/PclData.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>



//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_CLUSTERS 4
typedef pcl::PointXYZ PointT;
std::string filename;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class ClusterExtractor
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];
    ros::Publisher point_pub[MAX_CLUSTERS];
    ros::Publisher pclData_pub;
    tf::TransformBroadcaster br;


public:
    ClusterExtractor()
        {
            ROS_DEBUG("Creating subscribers and publishers");
            cloud_sub = n_.subscribe("/outlier/cutoff/output", 10, &ClusterExtractor::cloudcb, this);
            pclData_pub = n_.advertise<inspector::PclData>("pclData", 1);
            br = tf::TransformBroadcaster();
            int i = 0;
            for (i=0; i<MAX_CLUSTERS; i++)
            {
                std::stringstream ss;
                ss << "/cluster_" << i+1 << "_cloud";
                cloud_pub[i] = n_.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);
                ss.str(std::string());
                ss << "/cluster_" << i+1 << "_point";
                point_pub[i] = n_.advertise<geometry_msgs::PointStamped>(ss.str(), 1);
            }
            return;
        }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
        {
            ROS_DEBUG("Filtered cloud receieved");
            sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

            // set time stamp and frame id
            ros::Time tstamp = ros::Time::now();

            // Convert to pcl
            ROS_DEBUG("Convert incoming cloud to pcl cloud");
            pcl::fromROSMsg(*scan, *cloud);


            ////////////////////////////////////////
            // STARTING CLUSTER EXTRACTION    //
            ////////////////////////////////////////
            ROS_DEBUG("Begin cluster extraction");

            // create a vector for storing the indices of the clusters
            std::vector<pcl::PointIndices> cluster_indices;

            // setup extraction:
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.01); // cm
            ec.setMinClusterSize (50);
            ec.setMaxClusterSize (5000);
            ec.setInputCloud (cloud);
            // perform cluster extraction
            ec.extract (cluster_indices);

            // run through the indices, create new clouds, and then publish them
            int j=0;
            int number_clusters=0;
            geometry_msgs::PointStamped pt;
            geometry_msgs::Point pt2;
            Eigen::Vector4f centroid;
            inspector::PclData pclData;

            for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end (); ++it)
            {
                float xmin = 10;
                float xmax = -10;
                float ymin = 10;
                float ymax = -10;
                number_clusters = (int) cluster_indices.size();
                ROS_DEBUG("Number of clusters found: %d",number_clusters);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                {
                    cloud_cluster->points.push_back(cloud->points[*pit]);
                    if (cloud->points[*pit].x < xmin) {
                      xmin = cloud->points[*pit].x;
                    }
                    if (cloud->points[*pit].x > xmax) {
                      xmax = cloud->points[*pit].x;
                    }

                    if (cloud->points[*pit].y < ymin) {
                      ymin = cloud->points[*pit].y;
                    }
                    if (cloud->points[*pit].y > ymax) {
                      ymax = cloud->points[*pit].y;
                    }

                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;



                // convert to rosmsg and publish:
                ROS_DEBUG("Publishing extracted cloud");
                pcl::toROSMsg(*cloud_cluster, *ros_cloud);
                ros_cloud->header.frame_id = scan->header.frame_id;
                if(j < MAX_CLUSTERS)
                {
                    cloud_pub[j].publish(ros_cloud);

                    // compute centroid and publish
                    pcl::compute3DCentroid(*cloud_cluster, centroid);
                    pt.point.x = centroid(0);
                    pt.point.y = centroid(1);
                    pt.point.z = centroid(2);
                    pt.header.stamp = scan->header.stamp;
                    pt.header.frame_id = scan->header.frame_id;
                    point_pub[j].publish(pt);

                    float height = ymax-ymin;
                    float width = xmax-xmin;
                    float ratio = width/height;
                    //push height, width, ratio to output message
		    pclData.height[j] = height;
		    pclData.width[j] = width;
		    pclData.ratio[j] = ratio;
		    
                    //pclData.height.push_back(height);
                    //pclData.width.push_back(width);
                    //pclData.ratio.push_back(ratio);

                    pt2.x = centroid(0);
                    pt2.y = centroid(1);
                    pt2.z = centroid(2);
		    // pclData.centroid.push_back(pt2);
		    pclData.centroid[j] = pt2;
                    pclData_pub.publish(pclData);

                    // let's send transforms as well:
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
                    tf::Quaternion q;
                    q.setRPY(0, 0, 0);
                    transform.setRotation(q);
                    std::stringstream ss;
                    ss << "cluster_" << j+1 << "_frame";
                    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), scan->header.frame_id, ss.str()));

                    //Transform points to Baxters POV
                    // geometry_msgs::Point transCentroid;
                    // //Rot x
                    // float** rotx = new float*[4];
                    // for(int i = 0; i < 4; ++i) {
                    //   trans[i] = new float[4];
                    // }
                    //
                    // //Rot y
                    // float** roty = new float*[4];
                    // for(int i = 0; i < 4; ++i) {
                    //   trans[i] = new float[4];
                    // }
                    //
                    // double pi = 3.1415926535897;
                    // float theta = pi/2;
                    // rotx = [[1,0,0,0],[0,cos(theta),-sin(theta),0],[0,sin(theta),cos(theta),0],[0,0,0,1]];
                    // roty = [[cos(theta),0,sin(theta),0],[0,1,0,0],[-sin(theta),0,cos(theta)],[0,0,0,1]];


                  }
                j++;
            }
            return;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_extractor");
    ClusterExtractor extractor;

    ros::spin();

    return 0;
}
