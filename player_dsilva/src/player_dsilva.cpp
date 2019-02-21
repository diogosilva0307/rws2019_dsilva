#include <ros/ros.h>
#include <iostream>
#include <vector>

using namespace std;

namespace rws_dsilva
{
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
    setTeamName(team_name);
  }

private:
};

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
      else
      {
        return false;
      }
    }
  }

  string team_name;
  vector<string> player_names;
  ros::NodeHandle nh;

private:
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

  rws_dsilva::Team team_red("red");
  // team_red.player_names.push_back("dsilva");
  // team_red.player_names.push_back("moliveira");

  while (ros::ok())
  {
    team_red.printInfo();
    if (team_red.playerBelongsToTeam("dsilva") == 1)
    {
      cout << "This player belongs to the red team. " << endl;
    }
    else
    {
      cout << "This player doesn't belong to the red team. " << endl;
    }
    ros::Duration(1).sleep();
  }

  return 0;
}
