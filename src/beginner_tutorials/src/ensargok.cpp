#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <list>

enum direction 
{
    up,
    down,
    right,
    left
};

struct TurtleMove
{
    TurtleMove(direction _dir, int _dist)
        : dir(_dir)
        , dist(_dist)
    {  }
    direction dir;
    int dist;

    geometry_msgs::Twist asTwist() const
    {
        geometry_msgs::Twist msg = {};
        if (dir == direction::up)
            msg.linear.y = 1.f * dist;
        else if (dir == direction::down)
            msg.linear.y = -1.f * dist;
        else if (dir == direction::right)
            msg.linear.x = 1.f * dist;
        else if (dir == direction::left)
            msg.linear.x = -1.f * dist;
        else 
            ROS_ERROR("Unknown direction: %d", dir);

        return msg;
    }
};

struct TurtleChar
{
    std::vector<TurtleMove> moves = {};

    TurtleChar& addMove(direction dir, int dist) 
    {
        moves.emplace_back(dir, dist);
        return *this;
    }

};

class TurtleWriter 
{
public:
    void consume(ros::Publisher& pub)
    {
        if (list.empty())
            return;

        if ( list.front().dist * 5 > iter++ )
            pub.publish(list.front().asTwist());
        else 
        {
            list.erase(list.begin());
            iter = 0;
        }
    }

    void addChar(const TurtleChar& ch)
    {
        // c++23 -> append_range
        for ( const TurtleMove& move : ch.moves )
            list.emplace_back(move);
    }

private:
    std::list<TurtleMove> list = {};
    int iter = 0;
};

TurtleWriter g_Writer;

int main(int argc, char** argv)
{
    TurtleChar ch1 = TurtleChar().addMove(direction::down, 2).addMove(direction::right, 1);
    ros::init(argc, argv, "ensargok");
    
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg = {};
    msg.linear.x = -1.f;

    int count = 0;
    while(ros::ok())
    {
        /*
        if ( count < 5 )
            cmd_vel_pub.publish(ch1.moves[0].asTwist());
        else if ( count < 10 )
            cmd_vel_pub.publish(ch1.moves[1].asTwist());
        */

        if ( count == 15 )
            g_Writer.addChar(ch1);
        
        g_Writer.consume(cmd_vel_pub);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }

    return 0;
}
