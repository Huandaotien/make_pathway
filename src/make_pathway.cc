#include <make_pathway/make_pathway.h>

namespace make_pathway
{

    MakePathwayNode::MakePathwayNode(tf2_ros::Buffer* const tf_buffer, const string& nodename):nodename_(nodename), nh_("~"), tf_buffer_(tf_buffer)
    {
        ROS_INFO("Name is %s", nodename_.c_str());
        userParams_ = new userParams();
        pathway = new Pathway();
        startPose = new Pose();
        if(LoadParams(userParams_, nodename_)){
            ROS_INFO("%s : Params is loaded", nodename_.c_str());
        }
        else ROS_ERROR("%s : Fail to load params", nodename_.c_str());
        pathway_maker_publisher_ = nh_.advertise<visualization_msgs::Marker>("pathway_maker",10);        
        path_msg_publisher_ = nh_.advertise<nav_msgs::Path>("pathway_data",10);
        service_servers_.push_back(nh_.advertiseService("start_create_path", &MakePathwayNode::HandleStartCreatePaths, this));
        service_servers_.push_back(nh_.advertiseService("stop_create_path", &MakePathwayNode::HandleStopCreatePaths, this));
        currentPoseSub_ = nh_.subscribe(userParams_->current_pose_topic_name, 100, &MakePathwayNode::HandleCurrentPose, this);
        odomSub_ = nh_.subscribe("/odom", 100, &MakePathwayNode::HandleOdomData, this);
        creating_path_timer_ = nh_.createTimer(ros::Duration(0.05),&MakePathwayNode::CreatingPathTimer_CB, this);
        publish_virtual_line_timer_ = nh_.createTimer(ros::Duration(0.05), &MakePathwayNode::PublishPathwayData, this);

        string pathway_fullfilename = userParams_->directory_to_save_paths + "/" + userParams_->pathway_filename;        
        if(loadPathwayData(pathway_fullfilename)) cout<< "Success in load pathway file: "<<pathway_fullfilename<<endl;
        else std::cout<<pathway_fullfilename<<" is not existed"<<std::endl;
        pathway->testprint();
        creating_path_timer_.stop();
        start_create_paths = false;
    }

    MakePathwayNode::~MakePathwayNode()
    {
        delete(userParams_);
        delete(pathway);
        delete(startPose);
    }

    bool MakePathwayNode::HandleStartCreatePaths(
        ::xloc_msgs::StartCreatePaths_::Request& request,
        ::xloc_msgs::StartCreatePaths_::Response& response)
    {
        if(!start_create_paths){
            boost::lock_guard<boost::mutex> lock(mutex_);
            startPose->setPose(current_pose_X, current_pose_Y, current_pose_Yaw);
            start_create_paths = true;
            creating_path_timer_.start();
            response.success = true;
            response.message = "Create paths is in progress.";
        }
        else
        {
            response.success = false;
            response.message = "Create paths is still running.";
        }
        return true;
    }

    bool MakePathwayNode::HandleStopCreatePaths(
        ::xloc_msgs::StopCreatePaths_::Request& request,
        ::xloc_msgs::StopCreatePaths_::Response& response)                    
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        if(start_create_paths){
            start_create_paths = false;
            creating_path_timer_.stop();
            startPose->setPose(0.0, 0.0, 0.0);
            pathway->isPathInitilized_ = false;
            pathway->isPathClear_ = false;
            pathway->syncPosesAndPath();
            std::string fileNameToSave = userParams_->directory_to_save_paths + "/" + userParams_->pathway_filename;
            pathway->SavePathAsFile(fileNameToSave);
            response.success = true;
            response.message = "Create paths is stopped.";
        }
        else{
            response.success = false;
            response.message = "Create paths hasn't started yet.";
        }
        return true;
    }

    void MakePathwayNode::CreatingPathTimer_CB(const ros::TimerEvent& timer_event)
    {
        if(start_create_paths)
        {
            if(!if_current_pose_msg_update)
            {
                geometry_msgs::Pose current_pose;
                if(getPose(userParams_->base_frame_id, userParams_->map_frame_id, current_pose))
                {
                    current_pose_X = current_pose.position.x;
                    current_pose_Y = current_pose.position.y;
                    current_pose_Yaw = getYaw(current_pose.orientation.x, 
                    current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
                    // ROS_WARN("current_pose_X: %f, current_pose_Y: %f, current_pose_Yaw: %f", current_pose_X, current_pose_Y, current_pose_Yaw);                
                } 
            }
            Pose currentPose(current_pose_X, current_pose_Y, current_pose_Yaw);
            pathway->ResetAndRecordPath(*startPose, currentPose, 2, 0.15, 0.05);                       
        }
    }

    void MakePathwayNode::PublishPathwayData(const ros::TimerEvent& timer_event)
    {
        nav_msgs::Path gui_path;
        gui_path.poses.resize((int)posesOnPathWay.size());
        gui_path.header.frame_id = userParams_->map_frame_id;
        gui_path.header.stamp = ros::Time::now();
        for(int i=0; i<gui_path.poses.size(); i++)
        {
            gui_path.poses[i].pose.position.x = posesOnPathWay[i].getX();
            gui_path.poses[i].pose.position.y = posesOnPathWay[i].getY();
            gui_path.poses[i].pose.position.z = 0.0;
            gui_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
        }
        path_msg_publisher_.publish(gui_path);
        // visualization_msgs::Marker PathwayMarker = MakePathWayMarker(posesOnPathWay, userParams_->map_frame_id);
        // pathway_maker_publisher_.publish(PathwayMarker);
    }

    void MakePathwayNode::HandleCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {        
        current_pose_X = msg->pose.position.x;
        current_pose_Y = msg->pose.position.y;
        current_pose_Yaw = getYaw(msg->pose.orientation.x, 
        msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        ROS_WARN("current_pose_X: %f, current_pose_Y: %f, current_pose_Yaw: %f", current_pose_X, current_pose_Y, current_pose_Yaw);
        if_current_pose_msg_update = true;
    }

    void MakePathwayNode::HandleOdomData(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // double odomPoseX = msg->pose.pose.position.x;
        // double odomPoseY = msg->pose.pose.position.y;
        // double odomPoseYaw =  getYaw(msg->pose.pose.orientation.x, 
        // msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);;
        // ROS_WARN("odom_pose_X: %f, odom_pose_Y: %f, odom_pose_Yaw: %f", odomPoseX, odomPoseY, odomPoseYaw);

        // current_pose_X = msg->pose.pose.position.x;
        // current_pose_Y = msg->pose.pose.position.y;
        // current_pose_Yaw =  getYaw(msg->pose.pose.orientation.x, 
        // msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);;
    }

    inline double MakePathwayNode::getYaw(double x, double y, double z, double w){
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }

    bool MakePathwayNode::loadPathwayData(const string& filename)
    {
        bool result = false;
        // Check if saved path file is existed
        ifstream file;
        file.open(filename);
        if(file){
            pathway->LoadPathFromFile(filename);
            pathway->syncPosesAndPath();
            posesOnPathWay = pathway->getPosesOnPath();
            result = true;
        }
        else
            result = false;
        return result;
    }

    visualization_msgs::Marker MakePathwayNode::MakePathWayMarker(vector<Pose> posesOnPathWay, string frame)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.ns = "pathway_marker_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        int size = (int)posesOnPathWay.size();
        marker.points.resize(size);
        marker.colors.resize(size);
        geometry_msgs::Point p;
        for (int i = 0; i < (int)posesOnPathWay.size(); ++i) {
            p.x = posesOnPathWay[i].getX();
            p.y = posesOnPathWay[i].getY();
            p.z = 0.0;
            if (i==0)
                c.r = 1.0, c.g = 0.0, c.b = 0.0;
            else
                c.r = 0.0, c.g = 1.0, c.b = 0.0;
            marker.points[i] = p;
            marker.colors[i] = c;
        }

        return marker;
    }

    bool MakePathwayNode::LoadParams(userParams* userParams_, const string& node_name){
        bool result = true;
        try{            
            int baudrate;
            if(!ros::param::get(node_name + "/directory_to_save_paths", userParams_->directory_to_save_paths)) result = false;
            if(!ros::param::get(node_name + "/pathway_filename", userParams_->pathway_filename)) result = false;
            if(!ros::param::get(node_name + "/current_pose_topic_name", userParams_->current_pose_topic_name)) result = false;
            if(!ros::param::get(node_name + "/map_frame_id", userParams_->map_frame_id)) result = false;
            if(!ros::param::get(node_name + "/base_frame_id", userParams_->base_frame_id)) result = false;
        }
        catch(const std::exception& ex)
        {
            std::cerr << "Fail to load user params caught: " << ex.what() << std::endl;
            result = false;
        }
        return result;
    }

    bool MakePathwayNode::getPose(std::string base_frame_id, std::string map_frame, geometry_msgs::Pose &pose)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {        
            transformStamped = tf_buffer_->lookupTransform(map_frame, base_frame_id, ros::Time(0), ros::Duration(0.5));
            pose.position.x = transformStamped.transform.translation.x;
            pose.position.y = transformStamped.transform.translation.y;

            // Extract rotation information (yaw)
            tf::Quaternion tf_quaternion;
            tf::quaternionMsgToTF(transformStamped.transform.rotation, tf_quaternion);
            // Convert the quaternion to a yaw angle (in radians)
            double poseYaw = tf::getYaw(tf_quaternion);
            pose.orientation = tf::createQuaternionMsgFromYaw(poseYaw);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        return true;
    }
};