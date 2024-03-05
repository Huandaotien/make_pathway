#ifndef MAKE_PATHWAY_MAKE_PATHWAY_H
#define MAKE_PATHWAY_MAKE_PATHWAY_H

#include <make_pathway/pathway.h>
#include <make_pathway/pose.h>

#include <iostream>
#include <vector>
#include <thread>
#include <boost/thread.hpp>
#include <memory>

// xloc_msgs
#include <xloc_msgs/StartCreatePaths_.h>
#include <xloc_msgs/StopCreatePaths_.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>



using namespace std;
namespace make_pathway
{
    struct userParams{
        userParams(){
            directory_to_save_paths = "/init/paths";             
            pathway_filename = "pathway.txt";
            current_pose_topic_name = "/amcl_pose";
            map_frame_id = "map";
            base_frame_id = "base_link";
        }
        string directory_to_save_paths;
        string pathway_filename;
        string current_pose_topic_name;
        string map_frame_id;
        string base_frame_id;
    };

    class MakePathwayNode
    {
        public:
            MakePathwayNode(tf2_ros::Buffer* const tf_buffer, const string& nodename);
            ~MakePathwayNode();
        private:
            bool HandleStartCreatePaths(::xloc_msgs::StartCreatePaths_::Request& request, ::xloc_msgs::StartCreatePaths_::Response& response);  

            bool HandleStopCreatePaths(::xloc_msgs::StopCreatePaths_::Request& request, ::xloc_msgs::StopCreatePaths_::Response& response);      

            void CreatingPathTimer_CB(const ros::TimerEvent& timer_event);
            void PublishPathwayData(const ::ros::TimerEvent& timer_event); 

            void HandleCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
            void HandleOdomData(const nav_msgs::Odometry::ConstPtr &msg);


            visualization_msgs::Marker MakePathWayMarker(vector<Pose> posesOnPathWay, string frame);

            bool loadPathwayData(const string& filename);

            inline double getYaw(double x, double y, double z, double w);

            bool getPose(std::string base_frame_id, std::string map_frame, geometry_msgs::Pose &pose);

            bool LoadParams(userParams* userParams_, const string& node_name);

            Pathway* pathway;   
            userParams* userParams_; 
            Pose* startPose;
            vector<Pose> posesOnPathWay;

            tf2_ros::Buffer* const tf_buffer_;

            double current_pose_X;
            double current_pose_Y;
            double current_pose_Yaw; 

            bool start_create_paths;   
            string nodename_;         

            ros::NodeHandle nh_;
            boost::mutex mutex_;
            ros::Publisher pathway_maker_publisher_;
            ros::Publisher path_msg_publisher_;
            ros::Subscriber currentPoseSub_;
            ros::Subscriber odomSub_;
            vector<ros::ServiceServer> service_servers_;
            ros::Timer creating_path_timer_;
            ros::Timer publish_virtual_line_timer_;
    };
};

#endif