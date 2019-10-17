#include <pcl_apps/points_concatenate_component.h>

namespace pcl_apps
{
    PointsConcatenateComponent::PointsConcatenateComponent(const rclcpp::NodeOptions & options)
    : Node("points_concatenate", options)
    {
        declare_parameter("num_input",2);
        get_parameter("num_input",num_input_);
    }

    void PointsConcatenateComponent::input
        (const PointCloud2::SharedPtr &in0, const PointCloud2::SharedPtr &in1,
          const PointCloud2::SharedPtr &in2, const PointCloud2::SharedPtr &in3,
          const PointCloud2::SharedPtr &in4, const PointCloud2::SharedPtr &in5,
          const PointCloud2::SharedPtr &in6, const PointCloud2::SharedPtr &in7)
        {
            assert(num_input_>=2 && num_input_<=8);
            switch(num_input_)
            {
                case 2:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    break;
                case 3:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    break;
                case 4:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    assert(in2->header.frame_id == in3->header.frame_id);
                    break;
                case 5:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    assert(in2->header.frame_id == in3->header.frame_id);
                    assert(in3->header.frame_id == in4->header.frame_id);
                    break;
                case 6:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    assert(in2->header.frame_id == in3->header.frame_id);
                    assert(in3->header.frame_id == in4->header.frame_id);
                    assert(in4->header.frame_id == in5->header.frame_id);
                    break;
                case 7:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    assert(in2->header.frame_id == in3->header.frame_id);
                    assert(in3->header.frame_id == in4->header.frame_id);
                    assert(in4->header.frame_id == in5->header.frame_id);
                    assert(in5->header.frame_id == in6->header.frame_id);
                    break;
                case 8:
                    assert(in0->header.frame_id == in1->header.frame_id);
                    assert(in1->header.frame_id == in2->header.frame_id);
                    assert(in2->header.frame_id == in3->header.frame_id);
                    assert(in3->header.frame_id == in4->header.frame_id);
                    assert(in4->header.frame_id == in5->header.frame_id);
                    assert(in5->header.frame_id == in6->header.frame_id);
                    assert(in6->header.frame_id == in7->header.frame_id);
                    break;
            }
        }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsConcatenateComponent)