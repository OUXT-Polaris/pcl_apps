#include <pcl_apps/points_concatenate_component.h>

namespace pcl_apps
{
    PointsConcatenateComponent::PointsConcatenateComponent(const rclcpp::NodeOptions & options)
    : Node("points_concatenate", options)
    {
        declare_parameter("num_input",2);
        get_parameter("num_input",num_input_);
        set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> params) 
            -> rcl_interfaces::msg::SetParametersResult
        {
            auto results = rcl_interfaces::msg::SetParametersResult();
            results.successful = false;
            for(auto param : params)
            {
                if(param.get_name() == "num_input")
                {
                    num_input_ = param.as_int();
                    results.successful = true;
                    results.reason = "num_input recieved";
                }
            }
            if(!results.successful)
            {
                results.reason = "param name does not match";
            }
            return results;
        });
    }

    /*
    void PointsConcatenateComponent::input
        (const sensor_msgs::msg::PointCloud2::SharedPtr &in0, const sensor_msgs::msg::PointCloud2::SharedPtr &in1, 
        const sensor_msgs::msg::PointCloud2::SharedPtr &in2, const sensor_msgs::msg::PointCloud2::SharedPtr &in3, 
        const sensor_msgs::msg::PointCloud2::SharedPtr &in4, const sensor_msgs::msg::PointCloud2::SharedPtr &in5, 
        const sensor_msgs::msg::PointCloud2::SharedPtr &in6, const sensor_msgs::msg::PointCloud2::SharedPtr &in7)
        {
            pcl::PCLPointCloud2 in0_pc;
            pcl_conversions::toPCL(*in0, in0_pc);
        }
    */
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsConcatenateComponent)