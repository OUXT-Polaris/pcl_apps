#include <pcl_apps/ndt_matching/ndt_matching_component.h>

namespace pcl_apps
{
    NdtMatchingComponent::NdtMatchingComponent(const rclcpp::NodeOptions & options)
    : Node("voxelgrid_filter", options)
    {
        /* Dynamic Parameters */
        declare_parameter("transform_epsilon",1.0);
        get_parameter("transform_epsilon",transform_epsilon_);
        declare_parameter("step_size",0.1);
        get_parameter("step_size",step_size_);
        declare_parameter("resolution",1.0);
        get_parameter("resolution",resolution_);
        declare_parameter("max_iterations",35);
        get_parameter("max_iterations",max_iterations_);
        set_on_parameters_set_callback(
        [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult 
        {
            auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
            for(auto param : params)
            {
                if(param.get_name() == "transform_epsilon")
                {
                    if(transform_epsilon_ >0)
                    {
                        transform_epsilon_ = param.as_double();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "transform epsilon must over 0";
                    }
                }
                if(param.get_name() == "step_size")
                {
                    if(step_size_ >0)
                    {
                        step_size_ = param.as_double();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "step size must over 0";
                    }
                }
                if(param.get_name() == "resolution")
                {
                    if(resolution_ >0)
                    {
                        resolution_ = param.as_double();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "resolution must over 0";
                    }
                }
                if(param.get_name() == "max_iterations")
                {
                    if(max_iterations_ >= 1)
                    {
                        max_iterations_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "resolution must over 1";
                    }
                }
            }
            if(!results->successful)
            {
                results->successful = false;
                results->reason = "";
            }
            return *results;
        }
        );
        /* Static Parameters */
        declare_parameter("reference_frame_id","map");
        declare_parameter("reference_frame_id",reference_frame_id_);
        declare_parameter("reference_cloud_topic",get_name() + std::string("/reference"));
        get_parameter("reference_cloud_topic",reference_cloud_topic_);
        declare_parameter("input_cloud_topic",get_name() + std::string("/input"));
        get_parameter("input_cloud_topic",input_cloud_topic_);
        reference_cloud_recieved_ = false;
        initial_pose_recieved_ = false;
        auto reference_cloud_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            initial_pose_recieved_ = false;
            reference_cloud_recieved_ = true;
            pcl::fromROSMsg(*msg,*reference_cloud_);
        };
        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            initial_pose_recieved_ = true;
            assert(msg->header.frame_id == reference_frame_id_);
            current_relative_pose_ = *msg;
        };
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
            pcl::fromROSMsg(*msg,*input_cloud);
            updateRelativePose(input_cloud);
        };
        rmw_qos_profile_t qos;
        qos.depth = 1;
        qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        sub_reference_cloud_  = 
            create_subscription<sensor_msgs::msg::PointCloud2>(reference_cloud_topic_, 
                reference_cloud_callback, qos);
        sub_input_cloud_ = 
            create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10, callback);
    }   

    void NdtMatchingComponent::updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        ndt_.setTransformationEpsilon(transform_epsilon_);
        ndt_.setStepSize(step_size_);
        ndt_.setResolution(resolution_);
        ndt_.setMaximumIterations(max_iterations_);
        ndt_.setInputSource(input_cloud);
        ndt_.setInputTarget(reference_cloud_);
        geometry_msgs::msg::Transform transform;
        Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast<float>();
        return;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingComponent)