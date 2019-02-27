#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

using namespace std;

float randomizePosition()
{
  srand(8754 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace rws_dsilva
{
class Team
{
public:
  Team(string team_name)
  {
    this->team_name = team_name;
    nh.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team " << team_name << " has players: " << endl;
    for (size_t i = 0; i < player_names.size(); i++)
    {
      cout << player_names[i] << endl;
    }
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_name == player_names[i])
      {
        return true;
      }
    }
    return false;
  }

  string team_name;
  vector<string> player_names;
  ros::NodeHandle nh;

private:
};

class Player
{
public:
  Player(string name)  // Constructor
  {
    this->name = name;
    this->team_name = team_name;
  }

  void setTeamName(string team_name)
  {
    if (team_name == "red" || team_name == "green" || team_name == "blue")
    {
      this->team_name = team_name;
    }
    else
    {
      cout << "Cannot set team name" << team_name << endl;
    }
  }

  void setTeamName(int team_index)
  {
    if (team_index == 0)
    {
      setTeamName("red");
    }
    else if (team_index == 1)
    {
      setTeamName("green");
    }
    else if (team_index == 2)
    {
      setTeamName("blue");
    }
    else
    {
      setTeamName("");
    }
  }

  string getTeamName(void)
  {
    return team_name;
  }

  string name;

private:
  string team_name;
};

class MyPlayer : public Player
{
public:
  MyPlayer(string name, string team_name) : Player(name)
  {
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");
    // Create a smart pointer to the RVIZ marker
    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = nh.advertise<visualization_msgs::Marker>("/bocas", 0);

    last_prey = "";
    last_hunter = "";

    if (team_red->playerBelongsToTeam(name))
    {
      team_mine = team_red;
      team_hunters = team_blue;
      team_preys = team_green;
    }

    else if (team_green->playerBelongsToTeam(name))
    {
      team_mine = team_green;
      team_hunters = team_red;
      team_preys = team_blue;
    }

    else if (team_blue->playerBelongsToTeam(name))
    {
      team_mine = team_blue;
      team_hunters = team_green;
      team_preys = team_red;
    }
    else
    {
      cout << "Something went wrong" << endl;
      ROS_INFO_STREAM("Team: " << team_mine->team_name);
    }
    setTeamName(team_mine->team_name);

    float sx = randomizePosition();
    float sy = randomizePosition();

    tf::Transform T1;
    T1.setOrigin(tf::Vector3(sx, sy, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    T1.setRotation(q);
    // Step 4: Define global movement
    tf::Transform Tglobal = T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", name));

    ros::Duration(0.1).sleep();
  }

  tuple<float, float> getDistanceAndAngleToArenaCenter()
  {
    return getDistanceAndAngleToPlayer("world");
  }

  tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform(name, other_player, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      return { 1000.0, 1000.0 };
    }

    float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());

    float a = atan2(T0.getOrigin().y(), T0.getOrigin().x());

    return { d, a };
  }
  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << name << " and my team is " << team_mine->team_name);

    ROS_INFO_STREAM("I'm hunting team " << team_preys->team_name << " and fleeing from team "
                                        << team_hunters->team_name);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    bool something_changed = false;
    ROS_INFO("Received a new msg");

    // Step 1 : Find out where i am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("/world", name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    vector<float> distance_to_preys;
    vector<float> angle_to_preys;
    vector<float> distance_to_hunters;
    vector<float> angle_to_hunters;
    tuple<float, float> t1 = getDistanceAndAngleToArenaCenter();

    float distance_to_arena_center = get<0>(t1);
    float angle_to_arena_center = get<1>(t1);

    // Step 2: define how i want to move
    for (size_t i = 0; i < msg->blue_alive.size(); i++)
    {
      // ROS_WARN_STREAM("Team preys: " << team_preys->player_names[i]);
      tuple<float, float> t = getDistanceAndAngleToPlayer(msg->blue_alive[i]);
      distance_to_preys.push_back(get<0>(t));
      angle_to_preys.push_back(get<1>(t));
    }

    for (size_t i = 0; i < msg->red_alive.size(); i++)
    {
      // ROS_WARN_STREAM("Team hunters: " << team_hunters->player_names[i]);
      tuple<float, float> t = getDistanceAndAngleToPlayer(msg->red_alive[i]);
      distance_to_hunters.push_back(get<0>(t1));
      angle_to_hunters.push_back(get<1>(t));
    }

    int idx_closest_prey = 0;
    float distance_closest_prey = 1000;
    int idx_closest_hunters = 0;
    float distance_closest_hunters = 1000;

    for (size_t i = 0; i < distance_to_preys.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_prey)
      {
        idx_closest_prey = i;
        distance_closest_prey = distance_to_preys[i];
      }
      // if (distance_to_hunters[i] < distance_closest_hunters)
      // {
      //   idx_closest_hunters = i;
      //   distance_closest_hunters = distance_to_hunters[i];
      // }
    }

    float dx = 10;
    float angle = angle_to_preys[idx_closest_prey];

    string prey = "";
    if (idx_closest_prey != -1)
    {
      prey = team_preys->player_names[idx_closest_hunters];
      if (prey != last_prey)
      {
        something_changed = true;
        last_prey = prey;
      }
    }
    if (distance_closest_hunters < distance_closest_prey)
    {
      dx = 10;
      angle = -angle_to_hunters[idx_closest_hunters];
    }
    else
    {
      dx = 10;
      angle = angle_to_preys[idx_closest_prey];
    }

    /*if (distance_to_arena_center > 7.0)
    {
      dx = 0.2;
      angle = angle_to_arena_center;
    }*/

    // Step 2.5: Check Validation

    float dx_max = msg->cheetah;
    dx > dx_max ? dx = dx_max : dx = dx;

    double amax = M_PI / 30;
    if (angle != 0)
    {
      fabs(angle) > fabs(amax) ? angle = amax * angle / fabs(angle) : angle = angle;
    }
    // Step 3: define local movement
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);
    // Step 4: Define global movement
    tf::Transform Tglobal = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", name));

    if (something_changed)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = name;
      marker.header.stamp = ros::Time();
      marker.ns = name;
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      //            marker.pose.position.x = 1;
      //            marker.pose.position.y = 1;
      //            marker.pose.position.z = 1;
      //            marker.pose.orientation.x = 0.0;
      //            marker.pose.orientation.y = 0.0;
      //            marker.pose.orientation.z = 0.0;
      //            marker.pose.orientation.w = 1.0;
      //            marker.scale.x = ;
      //            marker.scale.y = 0.1;
      marker.scale.z = 0.5;
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.text = "Vais morrer " + prey;
      marker.lifetime = ros::Duration(2);

      // only if using a MESH_RESOURCE marker type:
      //            marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      vis_pub->publish(marker);
    }
  }

  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;

  string last_prey;
  string last_hunter;

  ros::NodeHandle nh;
  // TF broadcaster
  tf::TransformBroadcaster br;
  // TF listener
  tf::TransformListener listener;

private:
  boost::shared_ptr<ros::Publisher> vis_pub;
};

}  // namespace rws_dsilva

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dsilva");

  ros::NodeHandle nh;

  rws_dsilva::MyPlayer player("dsilva", "red");
  // player.setTeamName("green");
  // player.setTeamName(1);
  cout << "Hello world from " << player.name << " of team " << player.getTeamName() << endl;

  // rws_dsilva::Team team_red("red");
  // team_red.player_names.push_back("dsilva");
  // team_red.player_names.push_back("moliveira");

  ros::Subscriber sub = nh.subscribe("/make_a_play", 100, &rws_dsilva::MyPlayer::makeAPlayCallback, &player);

  player.printInfo();
  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 1;
}