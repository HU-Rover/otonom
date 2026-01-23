#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <grid_map_costmap_2d/costmap_2d_converter.hpp>

using std::placeholders::_1;

class GridMapToCostmapNode : public rclcpp::Node
{
public:
  GridMapToCostmapNode()
  : Node("grid_map_to_costmap_node")
  {
    gridmap_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping_node/elevation_map_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&GridMapToCostmapNode::gridMapCallback, this, _1));

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", qos);

    RCLCPP_INFO(this->get_logger(), "GridMap → Costmap2D → OccupancyGrid node started");
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);

    const std::string layer = "traversability";
    if (!grid_map.exists(layer)) {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' not found", layer.c_str());
      return;
    }

    // ---- Convert GridMap → Costmap2D ----
    nav2_costmap_2d::Costmap2D costmap;
    grid_map::Costmap2DConverter<grid_map::GridMap> converter;

    converter.initializeFromGridMap(grid_map, costmap);
    

    if (!converter.setCostmap2DFromGridMap(grid_map, layer, costmap)) {
      RCLCPP_ERROR(this->get_logger(), "%s", converter.errorMessage().c_str());
      return;
    }

    // ---- Convert Costmap2D → OccupancyGrid ----
    nav_msgs::msg::OccupancyGrid og;
    og.header.stamp = msg->header.stamp;
    og.header.frame_id = "map";

    og.info.resolution = costmap.getResolution();
    og.info.width  = costmap.getSizeInCellsX();
    og.info.height = costmap.getSizeInCellsY();

    og.info.origin.position.x = costmap.getOriginX();
    og.info.origin.position.y = costmap.getOriginY();
    og.info.origin.position.z = 0.0;
    og.info.origin.orientation.w = 1.0;

    const unsigned char* costmap_data = costmap.getCharMap();
    og.data.resize(og.info.width * og.info.height);

    for (unsigned int i = 0; i < og.data.size(); ++i) {
      unsigned char c = costmap_data[i];

      if (c == nav2_costmap_2d::NO_INFORMATION) {
        og.data[i] = -1;
      } else if (c >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        og.data[i] = 100;
      } else {
        og.data[i] = 0;
      }
    }

    map_pub_->publish(og);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapToCostmapNode>());
  rclcpp::shutdown();
  return 0;
}

