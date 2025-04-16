#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "turtlesim/SetPen.h"
#include <vector>
#include <list>

struct TurtleMove
{
    geometry_msgs::Twist twist;
    bool penDown = true;
};

struct TurtleChar
{
    std::vector<TurtleMove> moves = {};

    TurtleChar& addMove(const geometry_msgs::Twist& tw, bool penDown = true) 
    {
        moves.push_back({tw, penDown});
        return *this;
    }

};

class TurtleWriter 
{
public:
    void consume(ros::Publisher& pub, ros::ServiceClient& client)
    {
        if (listMoves.empty())
            return;

        if ( listMoves.front().penDown != m_penDown )
            togglePen(client);

        if ( 5 > iter++ )
            pub.publish(listMoves.front().twist);
        else 
        {
            listMoves.erase(listMoves.begin());
            iter = 0;
        }
    }

    void addChar(const TurtleChar& ch)
    {
        // c++23 -> append_range
        for ( const auto& move : ch.moves )
            listMoves.emplace_back(move);
    }

    void togglePen(ros::ServiceClient& client)
    {
        bool next = !m_penDown;

        ROS_WARN("next pen: %d", next);

        turtlesim::SetPen srv = {};
        srv.request.r = 255.f;
        srv.request.g = 255.f;
        srv.request.b = 255.f;
        srv.request.width = 2.f;
        srv.request.off = !next;

        client.call(srv);
        
        m_penDown = next;
    }

private:
    std::list<TurtleMove> listMoves = {};
    int iter = 0;
    bool m_penDown = true;
};

TurtleWriter g_Writer;

geometry_msgs::Twist createTwist(float x, float y, float z)
{
    geometry_msgs::Twist msg = {};
    msg.linear.x = x;
    msg.linear.y = y;
    msg.linear.z = z;
    return msg;
}

TurtleChar testA()
{
    TurtleChar ch = {};

    geometry_msgs::Twist leg1 = {};
    leg1.linear.x = 1.f;
    leg1.linear.y = 3.f;

    ch.addMove(leg1, true);

    geometry_msgs::Twist leg2 = {};
    leg2.linear.x = 0.5f;
    leg2.linear.y = -1.5f;

    ch.addMove(leg2);

    geometry_msgs::Twist leg3;
    leg3.linear.x = -1.f;
    ch.addMove(leg3);
    leg3.linear.x = 1.f;
    ch.addMove(leg3);

    ch.addMove(leg2);

    geometry_msgs::Twist end = {};
    ch.addMove(end);

    return ch;
}

TurtleChar charB()
{
    TurtleChar ch = {};
    geometry_msgs::Twist msg;
    ch.addMove(createTwist(0.f, 3.f, 0.f));
    ch.addMove(createTwist(1.0f, 0.f, 0.f));
    ch.addMove(createTwist(0, -1.5f, 0.f));
    ch.addMove(createTwist(-1.0f, 0.f, 0.f));
    ch.addMove(createTwist(1.5f, 0.f, 0.f));
    ch.addMove(createTwist(0, -1.5f, 0.f));
    ch.addMove(createTwist(-1.5f, 0.f, 0.f));
    ch.addMove(createTwist(1.5f, 0.f, 0.f));
    ch.addMove(createTwist(0.f, 0.f, 0.f));
    return ch;
}

TurtleChar charC() 
{
    TurtleChar ch = {};
    ch.addMove(createTwist(2.f, 3.f, 0), false);
    geometry_msgs::Twist msg = {};
    msg.linear.x = 0.f;
    msg.angular.z = -5.f;
    ch.addMove(msg);
    msg.linear.x = 4.f;
    msg.angular.z = 5.f;
    ch.addMove(msg, true);
    ch.addMove(createTwist(0.f, 0.f, 0.f));
    return ch;
}

TurtleChar charD()
{
    TurtleChar ch;
    ch.addMove(createTwist(0.f, 1.5f, 0));
    ch.addMove(createTwist(1.5f, 0.f, 0.f));
    ch.addMove(createTwist(0.f, 1.5f, 0.f));
    ch.addMove(createTwist(0.f, -3.f, 0.f));
    ch.addMove(createTwist(-1.5f, 0.f, 0.f));
    ch.addMove(createTwist(1.5f, 0, 0));
    ch.addMove(createTwist(0.f, 0.f, 0.f));
    return ch;
}

void turtleWriteCallback(const std_msgs::String& msg)
{
    static TurtleChar A = testA();
    static TurtleChar B = charB();
    static TurtleChar C = charC();
    static TurtleChar D = charD();
    ROS_INFO("I heard: [%s]", msg.data.c_str());
    const char* data = msg.data.c_str();

    while (*data) 
    {
        if ( *data == 'A' )
            g_Writer.addChar(A);
        else if ( *data == 'B' )
            g_Writer.addChar(B);
        else if ( *data == 'C' )
            g_Writer.addChar(C);
        else if ( *data == 'D' )
            g_Writer.addChar(D);
        ++data;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ensargok");
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::ServiceClient client = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    ros::Subscriber sub = n.subscribe("turtle_writer", 1000, turtleWriteCallback);

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok())
    {
        g_Writer.consume(cmd_vel_pub, client);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }

    return 0;
}
