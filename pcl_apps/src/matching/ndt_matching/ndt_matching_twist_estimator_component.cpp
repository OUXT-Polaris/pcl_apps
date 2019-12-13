#include <pcl_apps/matching/ndt_matching/ndt_matching_twist_estimator_component.h>

namespace pcl_apps
{
    NdtMatchingTwistEstimatorComponent::NdtMatchingTwistEstimatorComponent(const rclcpp::NodeOptions & options)
    : Node("ndt_matching_twist_estimator", options)
    {
        /* Static Parameters */
        declare_parameter("input_cloud_topic",get_name() + std::string("/input"));
        get_parameter("input_cloud_topic",input_cloud_topic_);
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
        // Setup Publisher
        std::string output_topic_name = get_name() + std::string("/current_relative_pose");
        current_twist_pub_ = 
            create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_name,10);
        // Setup Subscriber
        buffer_ = boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::Ptr>(2);
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
            pcl::fromROSMsg(*msg,*input_cloud);
            buffer_.push_back(input_cloud);
            boost::optional<geometry_msgs::msg::TwistStamped> twist = estimateCurrentTwist();
            if(twist)
            {
                current_twist_pub_->publish(twist.get());
            }
        };
        sub_input_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10, callback);
    }

    boost::optional<geometry_msgs::msg::TwistStamped> NdtMatchingTwistEstimatorComponent::estimateCurrentTwist()
    {
        if(buffer_.size() == 2)
        {
            ndt_.setTransformationEpsilon(transform_epsilon_);
            ndt_.setStepSize(step_size_);
            ndt_.setResolution(resolution_);
            ndt_.setMaximumIterations(max_iterations_);
            ndt_.setInputSource(buffer_[1]);
            ndt_.setInputTarget(buffer_[0]);
            geometry_msgs::msg::Transform transform;
        }
        return boost::none;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingTwistEstimatorComponent)