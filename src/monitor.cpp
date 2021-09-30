#include <unistd.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

// a simple node that will run on almost anything
// specifically built to test the ans
// publish the system hostname and anything else sane to monitor on a bare linux box.
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace monitoring
{

class monitor : public rclcpp::Node
{
  public:
  monitor(rclcpp::NodeOptions options);
  void publish_stuff();

  rclcpp::TimerBase::SharedPtr pubtimer;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub;
};


monitor::monitor(rclcpp::NodeOptions options) :
  Node("monitor", options)
{
  pubtimer = create_wall_timer(1000ms, std::bind(&monitor::publish_stuff, this));
  pub = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
}


double fetch_temperature(){
  std::string temp_path = "/sys/class/thermal/thermal_zone0/temp";
  char temp_str[10];
  int temp_millidegrees;
  int fd = open(temp_path.c_str(), O_RDONLY);
  /*size_t bytes_read = */read(fd, temp_str, 10);
  close(fd);
  // maybe error handling
  sscanf(temp_str, "%i", &temp_millidegrees);
  return ((double) temp_millidegrees)/1000.0;
}

void monitor::publish_stuff(){
  rclcpp::LoanedMessage<sensor_msgs::msg::Temperature> temperature =
    pub->borrow_loaned_message();

  // fetch hostname
  const size_t len = 100;
  char hostname[len];
  /*int err = */gethostname(hostname, len);
  temperature.get().header.stamp = now();
  temperature.get().header.frame_id = std::string(hostname);
  temperature.get().temperature = fetch_temperature();

  // publish
  pub->publish(std::move(temperature));
}

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<monitoring::monitor>(options));
    rclcpp::shutdown();
    return 0;
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(monitoring::monitor)
