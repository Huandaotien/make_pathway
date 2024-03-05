#include <make_pathway/make_pathway.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

using namespace std;
using namespace make_pathway;

MakePathwayNode* node;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_pathway_node");
    ros::start();
    string nodename = ros::this_node::getName();
    tf2_ros::Buffer tfBuffer(ros::Duration(10.));
    tf2_ros::TransformListener tf(tfBuffer);
    node = new MakePathwayNode(&tfBuffer,nodename);
    ros::spin();
    delete(node);
    ros::shutdown();
    return 0;
}