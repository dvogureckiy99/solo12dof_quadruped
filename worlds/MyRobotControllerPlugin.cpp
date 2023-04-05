#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
    class MyRobotControllerPlugin : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            this->model = _parent;

            std::string robotNamespace;
            if (_sdf->HasElement("robotNamespace"))
                robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
            else
                robotNamespace = this->model->GetName();

            std::string topicName;
            if (_sdf->HasElement("topicName"))
                topicName = _sdf->GetElement("topicName")->Get<std::string>();
            else
                topicName = "/cmd_vel";

            ros::NodeHandle nodeHandle(robotNamespace);

            this->publisher = nodeHandle.advertise<geometry_msgs::Twist>(topicName, 1);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyRobotControllerPlugin::OnUpdate, this, _1));
        }

        void OnUpdate()
        {
            // Get the current time
            common::Time current_time = this->world->SimTime();

            // Calculate the time since the last update
            double dt = (current_time - last_update_time).Double();

            // If the time since the last update is too small, skip this update
            if (dt < update_period) {
                return;
            }

            // Update the last update time to the current time
            last_update_time = current_time;

            // Get the current pose of the robot
            ignition::math::Pose3d current_pose = this->model->WorldPose();

            // Calculate the new pose of the robot using your control algorithm
            ignition::math::Pose3d new_pose = MyControlAlgorithm(current_pose, dt);

            // Set the new pose of the robot
            this->model->SetWorldPose(new_pose);
        }

        private: physics::ModelPtr model;
        private: ros::Publisher publisher;
        private: event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(MyRobotControllerPlugin)
}
