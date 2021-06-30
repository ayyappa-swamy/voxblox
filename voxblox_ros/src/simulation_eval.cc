#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "voxblox_ros/simulation_server.h"

namespace voxblox {
class SimulationServerImpl : public voxblox::SimulationServer {
 public:
  SimulationServerImpl(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
      : SimulationServer(nh, nh_private) {}

  void prepareWorld() {
    CHECK_NOTNULL(world_);
    // world_->addObject(std::unique_ptr<Object>(
    //     new Sphere(Point(0.0, 0.0, 2.0), 2.0, Color::Red())));
    world_->setBounds(Point(-100.0, -100.0, -10.0), Point(100.0, 100.0, 50.0));

    // world_->addObject(std::unique_ptr<Object>(
    //     new CuboidObject(Point(0.0, 0.0, 2.0), Point(0.5, 0.5, 0.0), 3.0/*breadth*/, 0.15/*width*/, Color::Red())));

    // Four plane segments similar to indoor sequence
    // world_->addObject(std::unique_ptr<Object>(
    //     new CuboidObject(Point(0.0, 3.0, 2.0), Point(0.0, 1.0, 0.0), 4.5/*breadth*/, 0.15/*width*/, Color::Gray())));

    // world_->addObject(std::unique_ptr<Object>(
    //     new CuboidObject(Point(0.0, -3.0, 2.0), Point(0.0, -1.0, 0.0), 4.5/*breadth*/, 0.15/*width*/, Color::Gray())));

    // world_->addObject(std::unique_ptr<Object>(
    //     new CuboidObject(Point(2.0, 0.0, 2.0), Point(1.0, 0.0, 0.0), 6.5/*breadth*/, 0.15/*width*/, Color::Gray())));

    world_->addObject(std::unique_ptr<Object>(
        new CuboidObject(Point(0.0, 0.0, 0.5), Point(-1.0, 0.0, 0.0), 1.5/*breadth*/, 0.15/*width*/, Color::Gray(), 5)));

    // world_->addObject(std::unique_ptr<Object>(new PlaneObject(
    //     Point(-2.0, -4.0, 2.0), Point(0, 1, 0), Color::White())));

    // world_->addObject(std::unique_ptr<Object>(
    //     new PlaneObject(Point(4.0, 0.0, 0.0), Point(-1, 0, 0), Color::Pink())));

    // world_->addObject(std::unique_ptr<Object>(
    //     new CuboidObject(Point(-3.0, 3.0, 4.0), Point(1.0, 3.0, 0.0), 1.0, 2.0, Color::Green())));

    // world_->addObject(std::unique_ptr<Object>(
    //     new Cube(Point(-4.0, 4.0, 2.0), Point(4, 4, 4), Color::Green())));

    // world_->addGroundLevel(0.03);

    world_->generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_->generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }
};

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::SimulationServerImpl sim_eval(nh, nh_private);

  sim_eval.run();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}
