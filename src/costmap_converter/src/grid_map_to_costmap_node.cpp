#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

class GridMapToOccupancyGrid : public rclcpp::Node
{
public:
  GridMapToOccupancyGrid()
  : Node("grid_map_to_occupancy_grid")
  {
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping_node/elevation_map_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&GridMapToOccupancyGrid::callback, this, _1));

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

    RCLCPP_INFO(this->get_logger(), "GridMap → OccupancyGrid node started");
  }

private:
  void callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);

    const std::string layer = "traversability";

    if (!grid_map.exists(layer)) {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' not found", layer.c_str());
      return;
    }

    const grid_map::Matrix& data = grid_map[layer];

    nav_msgs::msg::OccupancyGrid og;
    og.header.stamp = msg->header.stamp;
    og.header.frame_id = "map";

    og.info.resolution = grid_map.getResolution();
    og.info.width      = grid_map.getSize()(0);
    og.info.height     = grid_map.getSize()(1);

    // GridMap center → OccupancyGrid bottom-left origin
    og.info.origin.position.x = grid_map.getPosition().x() - grid_map.getLength().x() / 2.0;
    og.info.origin.position.y = grid_map.getPosition().y() - grid_map.getLength().y() / 2.0;
    og.info.origin.position.z = 0.0;
    og.info.origin.orientation.w = 1.0;

    og.data.resize(og.info.width * og.info.height, -1);

    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it)
    {
      const grid_map::Index idx(*it);
      float val = data(idx(0), idx(1));

      // Convert GridMap index to OccupancyGrid flat index
      // GridMap: row=X(0), col=Y(1), origin at CENTER, X forward, Y left
      // OccupancyGrid: row-major, origin at BOTTOM-LEFT
      // With this (180° rotation = flip both axes):
      unsigned int mx = og.info.width  - 1 - idx(0);
      unsigned int my = og.info.height - 1 - idx(1);
      unsigned int flat = my * og.info.width + mx;

      if (flat >= og.data.size()) continue;

      if (std::isnan(val)) {
        og.data[flat] = -1;   // unknown
      } else if (val < 0.3f) {
        og.data[flat] = 100;  // obstacle
      } else {
        og.data[flat] = 0;    // free
      }
    }

    pub_->publish(og);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapToOccupancyGrid>());
  rclcpp::shutdown();
  return 0;
}
