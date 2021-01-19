#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Quaternion.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/String.h>
#include <social_worlds/TaskCommand.h>
#include <social_worlds/ActorTrajectory.h>

#include <unistd.h>
#include <iostream>

#define deg2rad(X) X*3.1415/180
#define rad2deg(X) X*180/3.1415

namespace gazebo
{
  class GAZEBO_VISIBLE TaskControl_Receptionist : public WorldPlugin
  {

    // ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    // service servers
    private: ros::ServiceServer srv_server;

    // service clients
    private: ros::ServiceClient srv_client_human_1;
    private: ros::ServiceClient srv_client_human_2;
    // private: ros::ServiceClient srv_client_human_3;
    private: ros::ServiceClient srv_client_human_4;
    private: ros::ServiceClient srv_client_human_5;

    // Publishers
    private: ros::Publisher pub_door_1;
    private: ros::Publisher pub_door_2;
    private: ros::Publisher pub_door_3;
    private: ros::Publisher pub_door_4;
    private: ros::Publisher pub_door_5;

    // actors
    private: physics::ActorPtr human_1;
    private: physics::ActorPtr human_2;
    // private: physics::ActorPtr human_3;
    private: physics::ActorPtr human_4;
    private: physics::ActorPtr human_5;

    // world
    private: physics::WorldPtr world;

    // state
    private: int state;

    public: TaskControl_Receptionist()
    {
    }

    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "Starting task simulation: Receptionist!" << std::endl;
      this->world = _world;
      this->state = 0;

      // start ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle());

      // Service server: task
      this->srv_server = this->rosNode->advertiseService(
          "/task_control/receptionist/command",
          &TaskControl_Receptionist::OnService_Start, this);

      // Service clients: actors
      this->srv_client_human_1 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_1/action");
      this->srv_client_human_2 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_2/action");
      // this->srv_client_human_3 = this->rosNode->serviceClient
      //     <social_worlds::ActorTrajectory>
      //     ("/actor_control/human_3/action");
      this->srv_client_human_4 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_4/action");
      this->srv_client_human_5 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_5/action");


      // Doors publishers
      this->pub_door_1 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_1/command", 1000);
      this->pub_door_2 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_2/command", 1000);
      this->pub_door_3 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_3/command", 1000);
      this->pub_door_4 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_4/command", 1000);
      this->pub_door_5 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_5/command", 1000);

      // Actors models
      this->human_1 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_1"));
      this->human_2 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_2"));
      // this->human_3 = boost::reinterpret_pointer_cast
      //     <physics::Actor>(this->world->ModelByName("human_3"));
      this->human_4 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_4"));
      this->human_5 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_5"));

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&TaskControl_Receptionist::QueueThread, this));

    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: bool OnService_Start(
        social_worlds::TaskCommand::Request &_req,
        social_worlds::TaskCommand::Response &_res)
    {
      // strat task
      std::cout << "Receptionist task: Start." << std::endl;
      this->world->SetPaused(false);

      if(_req.command.data == "start")
      {
        // open inside doors
        std_msgs::String msg;
        msg.data = "open";
        this->pub_door_5.publish(msg);
      }


      // Para primeira pessoa
      else if(_req.command.data == "p1")
      {
        std::cout << "Receptionist task: Phase 1." << std::endl;
        this->state = 1;
        this->enter_hause(this->srv_client_human_2);
      }
      else if(_req.command.data == "p2")
      {
        std::cout << "Receptionist task: Phase 2." << std::endl;
        this->state = 2;
        this->turn_right(this->srv_client_human_2);
      }
      else if(_req.command.data == "p3")
      {
        std::cout << "Receptionist task: Phase 3." << std::endl;
        this->state = 3;
        this->follow(this->srv_client_human_2);
      }
      else if(_req.command.data == "p4")
      {
        std::cout << "Receptionist task: Phase 4." << std::endl;
        this->state = 4;
        this->armchair(this->srv_client_human_2);
      }
      else if(_req.command.data == "p5")
      {
        std::cout << "Receptionist task: Phase 5." << std::endl;
        this->state = 5;
        this->arm_to_john(this->srv_client_human_2);
      }
      
      
      // Para segunda pessoa
      // else if(_req.command.data == "p11")
      // {
      //   std::cout << "Receptionist task: Phase 11." << std::endl;
      //   this->state =11;
      //   this->enter_hause_2(this->srv_client_human_3);
      // }
      // else if(_req.command.data == "p12")
      // {
      //   std::cout << "Receptionist task: Phase 12." << std::endl;
      //   this->state = 12;
      //   this->turn_right_2(this->srv_client_human_3);
      // }
      // else if(_req.command.data == "p13")
      // {
      //   std::cout << "Receptionist task: Phase 13." << std::endl;
      //   this->state = 13;
      //   this->follow_2(this->srv_client_human_3);
      // }
      // else if(_req.command.data == "p14")
      // {
      //   std::cout << "Receptionist task: Phase 14." << std::endl;
      //   this->state = 14;
      //   this->empty_2(this->srv_client_human_3);
      // }
      // else if(_req.command.data == "p15")
      // {
      //   std::cout << "Receptionist task: Phase 15." << std::endl;
      //   this->state = 15;
      //   this->armchair_2(this->srv_client_human_3);
      // }
      // else if(_req.command.data == "p16")
      // {
      //   std::cout << "Receptionist task: Phase 16." << std::endl;
      //   this->state = 16;
      //   this->arm_to_empty(this->srv_client_human_3);
      // }

      // Para quinta pessoa
      else if(_req.command.data == "p11")
      {
        std::cout << "Receptionist task: Phase 11." << std::endl;
        this->state =11;
        this->enter_hause_2(this->srv_client_human_5);
      }
      else if(_req.command.data == "p12")
      {
        std::cout << "Receptionist task: Phase 12." << std::endl;
        this->state = 12;
        this->turn_right_2(this->srv_client_human_5);
      }
      else if(_req.command.data == "p13")
      {
        std::cout << "Receptionist task: Phase 13." << std::endl;
        this->state = 13;
        this->follow_2(this->srv_client_human_5);
      }
      else if(_req.command.data == "p14")
      {
        std::cout << "Receptionist task: Phase 14." << std::endl;
        this->state = 14;
        this->empty_2(this->srv_client_human_5);
      }
      else if(_req.command.data == "p15")
      {
        std::cout << "Receptionist task: Phase 15." << std::endl;
        this->state = 15;
        this->armchair_2(this->srv_client_human_5);
      }
      else if(_req.command.data == "p16")
      {
        std::cout << "Receptionist task: Phase 16." << std::endl;
        this->state = 16;
        this->arm_to_empty(this->srv_client_human_5);
      }

      // Para terceira pessoa
      else if(_req.command.data == "p21")
      {
        std::cout << "Receptionist task: Phase 21." << std::endl;
        this->state =21;
        this->enter_hause_3(this->srv_client_human_4);
      }
      else if(_req.command.data == "p22")
      {
        std::cout << "Receptionist task: Phase 22." << std::endl;
        this->state = 22;
        this->turn_right_3(this->srv_client_human_4);
      }
      else if(_req.command.data == "p23")
      {
        std::cout << "Receptionist task: Phase 23." << std::endl;
        this->state = 23;
        this->follow_3(this->srv_client_human_4);
      }
      else if(_req.command.data == "p24")
      {
        std::cout << "Receptionist task: Phase 24." << std::endl;
        this->state = 24;
        this->empty_3(this->srv_client_human_4);
      }
      else if(_req.command.data == "p25")
      {
        std::cout << "Receptionist task: Phase 15." << std::endl;
        this->state = 25;
        this->armchair_3(this->srv_client_human_4);
      }
      else if(_req.command.data == "p26")
      {
        std::cout << "Receptionist task: Phase 15." << std::endl;
        this->state = 26;
        this->to_john(this->srv_client_human_4);
      }
      return true;
    }


    private: void enter_hause(ros::ServiceClient srv_human)
    {
      // variables
      geometry_msgs::Pose p;
      std_msgs::String msg;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-6.446584, 3.443701, 1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // open entrance door
      msg.data = "open";
      this->pub_door_5.publish(msg);
      usleep(3000000); // microseconds

      // enter home
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.2836584);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_2.call(trajectory);
      // trajectory.request.animation.data = "walking";
      // for (size_t i = 0; i < 10; i++) {
      //   position.X(position.X()+0.5);
      //   p.position.x = position.X();
      //   trajectory.request.waypoints.push_back(p);
      // }
      // srv_human.call(trajectory);


      // close entrance door
      // usleep(3000000); // microseconds
      // msg.data = "close";
      // this->pub_door_1.publish(msg);
      
      usleep(3000000); // microseconds
      msg.data = "close";
      this->pub_door_5.publish(msg);

    }

    private: void enter_hause_2(ros::ServiceClient srv_human)
    {
      // variables
      geometry_msgs::Pose p;
      std_msgs::String msg;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-6.446584, 3.443701, 1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // open entrance door
      msg.data = "open";
      this->pub_door_5.publish(msg);
      usleep(3000000); // microseconds

      // enter home
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.2836584);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_5.call(trajectory);
          
      usleep(3000000); // microseconds
      msg.data = "close";
      this->pub_door_5.publish(msg);

    }

    private: void enter_hause_3(ros::ServiceClient srv_human)
    {
      // variables
      geometry_msgs::Pose p;
      std_msgs::String msg;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-6.446584, 3.443701, 1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // open entrance door
      msg.data = "open";
      this->pub_door_5.publish(msg);
      usleep(3000000); // microseconds

      // enter home
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.2836584);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_4.call(trajectory);
          
      usleep(3000000); // microseconds
      msg.data = "close";
      this->pub_door_5.publish(msg);

    }

    private: void follow(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-3.628388,2.002968,1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.4479789);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_2.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void follow_2(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-3.628388,2.002968,1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.4479789);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_5.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void follow_3(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 90;
      ignition::math::Vector3<double> position(-3.628388,2.002968,1.050000);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.4479789);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void turn_right(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(-3.61, 3.43, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.Y(position.Y() - 0.143);
        p.position.y = position.Y();   
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_2.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void turn_right_2(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(-3.61, 3.43, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.Y(position.Y() - 0.143);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_5.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void turn_right_3(ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(-3.61, 3.43, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.Y(position.Y() - 0.143);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void john (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.284972);
        p.position.x = position.X();
        position.Y(position.Y()+0.0521744);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void armchair (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.283557);
        p.position.x = position.X();
        position.Y(position.Y() - 0.0948868);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_2.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void armchair_2 (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.283557);
        p.position.x = position.X();
        position.Y(position.Y() - 0.0948868);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_5.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void armchair_3 (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.283557);
        p.position.x = position.X();
        position.Y(position.Y() - 0.0948868);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void empty_2 (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.116043);
        p.position.x = position.X();
        position.Y(position.Y()+0.2229181);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_5.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void empty_3 (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.116043);
        p.position.x = position.X();
        position.Y(position.Y()+0.2229181);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }

    private: void arm_to_john (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(3.686971, 0.975938, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.Y(position.Y()+0.1470612);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_2.call(trajectory);
      // srv_human.call(trajectory);
    }



    private: void arm_to_empty (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 0;
      ignition::math::Vector3<double> position(3.686971, 0.975938, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()-0.1675134);
        p.position.x = position.X();
        position.Y(position.Y()+0.3178049);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_5.call(trajectory);

    }

      private: void to_john (ros::ServiceClient srv_human)
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = -90;
      ignition::math::Vector3<double> position(0.851401, 1.924806, 1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.X(position.X()+0.2849723);
        p.position.x = position.X();
        position.Y(position.Y() +0.0521744);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }

      this->srv_client_human_4.call(trajectory);
      // srv_human.call(trajectory);
    }
      // srv_human.call(trajectory);
    

    // private: void sit_1()
    // {
    //   // variables
    //   geometry_msgs::Pose p;
    //   social_worlds::ActorTrajectory trajectory;
    //   double angle = 180;
    //   ignition::math::Vector3<double> position(2.7,0,1.05);
    //   ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
    //   p.position.x = position.X();
    //   p.position.y = position.Y();
    //   p.position.z = position.Z();
    //   p.orientation.x = orientation.X();
    //   p.orientation.y = orientation.Y();
    //   p.orientation.z = orientation.Z();
    //   p.orientation.w = orientation.W();

    //   // sit_down
    //   trajectory.request.animation.data = "walking";
    //   for (size_t i = 0; i < 2; i++) {
    //     position.Y(position.Y()+0.5);
    //     p.position.y = position.Y();
    //     trajectory.request.waypoints.push_back(p);
    //   }
    //   for (size_t i = 0; i < 2; i++) {
    //     position.X(position.X()+0.5);
    //     p.position.x = position.X();
    //     trajectory.request.waypoints.push_back(p);
    //   }
    //   for (size_t i = 0; i < 3; i++) {
    //     position.Y(position.Y()+0.5);
    //     p.position.y = position.Y();
    //     trajectory.request.waypoints.push_back(p);
    //   }
    //   angle = -90;
    //   orientation.Euler(deg2rad(90),deg2rad(0),deg2rad(angle));
    //   p.orientation.x = orientation.X();
    //   p.orientation.y = orientation.Y();
    //   p.orientation.z = orientation.Z();
    //   p.orientation.w = orientation.W();
    //   trajectory.request.waypoints.push_back(p);
    //   this->srv_client_human_2.call(trajectory);

    // }

    private: void sit_2()
    {
      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 180;
      ignition::math::Vector3<double> position(2.7,0,1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // sit_down
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 4; i++) {
        position.Y(position.Y()+1.0);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      angle = 0;
      orientation.Euler(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();
      trajectory.request.waypoints.push_back(p);
      this->srv_client_human_5.call(trajectory);

    }

    // private: void sit_3()
    // {
    //   // variables
    //   geometry_msgs::Pose p;
    //   social_worlds::ActorTrajectory trajectory;
    //   double angle = 180;
    //   ignition::math::Vector3<double> position(2.7,0,1.05);
    //   ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
    //   p.position.x = position.X();
    //   p.position.y = position.Y();
    //   p.position.z = position.Z();
    //   p.orientation.x = orientation.X();
    //   p.orientation.y = orientation.Y();
    //   p.orientation.z = orientation.Z();
    //   p.orientation.w = orientation.W();

    //   // sit_down
    //   trajectory.request.animation.data = "walking";
    //   for (size_t i = 0; i < 4; i++) {
    //     position.Y(position.Y()+1.0);
    //     p.position.y = position.Y();
    //     trajectory.request.waypoints.push_back(p);
    //   }
    //   angle = 0;
    //   orientation.Euler(deg2rad(90),deg2rad(0),deg2rad(angle));
    //   p.orientation.x = orientation.X();
    //   p.orientation.y = orientation.Y();
    //   p.orientation.z = orientation.Z();
    //   p.orientation.w = orientation.W();
    //   trajectory.request.waypoints.push_back(p);
    //   this->srv_client_human_4.call(trajectory);

    // }

  };

  GZ_REGISTER_WORLD_PLUGIN(TaskControl_Receptionist)
}
