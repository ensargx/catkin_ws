#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <list>

struct TurtleChar
{
    std::vector<geometry_msgs::Twist> moves = {};

    TurtleChar& addMove(const geometry_msgs::Twist& tw) 
    {
        moves.push_back(tw);
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

        if ( 5 > iter++ )
            pub.publish(list.front());
        else 
        {
            list.erase(list.begin());
            iter = 0;
        }
    }

    void addChar(const TurtleChar& ch)
    {
        // c++23 -> append_range
        for ( const geometry_msgs::Twist& move : ch.moves )
            list.emplace_back(move);
    }

private:
    std::list<geometry_msgs::Twist> list = {};
    int iter = 0;
};

TurtleWriter g_Writer;

TurtleChar testA()
{
    TurtleChar ch = {};

    geometry_msgs::Twist leg1 = {};
    leg1.linear.x = 1.f;
    leg1.linear.y = 3.f;

    ch.moves.push_back(leg1);

    geometry_msgs::Twist leg2 = {};
    leg2.linear.x = 0.5f;
    leg2.linear.y = -1.5f;

    ch.moves.push_back(leg2);

    geometry_msgs::Twist leg3;
    leg3.linear.x = -1.f;
    ch.moves.push_back(leg3);
    leg3.linear.x = 1.f;
    ch.moves.push_back(leg3);

    ch.moves.push_back(leg2);

    geometry_msgs::Twist end = {};
    ch.moves.push_back(end);

    return ch;
}

int main(int argc, char** argv)
{
    TurtleChar ch1 = testA();
    ros::init(argc, argv, "ensargok");
    
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg = {};
    msg.linear.x = -1.f;

    int count = 0;
    while(ros::ok())
    {
        if ( count == 15 )
            g_Writer.addChar(ch1);
        
        g_Writer.consume(cmd_vel_pub);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }

    return 0;
}
