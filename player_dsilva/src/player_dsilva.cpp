#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>

using namespace std;

float randomizePosition()
{
  srand(8754 * time(NULL)); // set initial seed value to 5323
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
  Player(string name) // Constructor
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
  }

  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << name << " and my team is " << team_mine->team_name);

    ROS_INFO_STREAM("I'm hunting team " << team_preys->team_name << " and fleeing from team "
                                        << team_hunters->team_name);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
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

    // Step 2: define how i want to move
    float dx = 0.8;
    float angle = M_PI / 6;
    // Step 3: define local movement
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);
    // Step 4: Define global movement
    tf::Transform Tglobal = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", name));
  }

  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;
  // TF broadcaster
  tf::TransformBroadcaster br;
  // TF listener
  tf::TransformListener listener;

private:
};

} // namespace rws_dsilva

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