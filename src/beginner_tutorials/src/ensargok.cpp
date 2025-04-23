#include "ros/console.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Pose.h"
#include <boost/math/policies/policy.hpp>
#include <cmath>
#include <utility>
#include <vector>
#include <math.h>

struct RelativePos
{
    float x, y;
    bool penDown = true;
};

struct TurtleMove
{
    geometry_msgs::Twist msg;
    bool penDown = true;
};

geometry_msgs::Twist TwistWr( float x, float y, float w )
{
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;
    return msg;
}

class CharMove
{
public:
    CharMove(std::vector<RelativePos>&& moves, float size = 1.f)
        : m_moves(moves)
        , m_moveIdx(0)
        , m_size(size)
    { }

    bool IsMovesDone()
    {
        return m_moves.size() == m_moveIdx;
    }

    geometry_msgs::Twist NextMove(float initialX, float initialY, float posX, float posY, float rotW)
    {
        if ( IsMovesDone() )
        {
            ROS_ERROR("This should never happen");
            return {};
        }

        ROS_DEBUG("rotW: %f", rotW);

        std::pair<float, float> target = GetNextTarget(initialX, initialY);

        ROS_DEBUG("position: %f, %f", posX, posY);
        ROS_DEBUG("target: %f, %f", target.first, target.second);

        double nextTurnRad = CalcualateRotation(posX, posY, target.first, target.second);
        ROS_DEBUG("nextTurnRad: %lf", nextTurnRad);

        double diffTurnRad = nextTurnRad - rotW;
        ROS_DEBUG("diffTurnRad = nextTurnRad - prewRot : %lf", diffTurnRad);

        if ( diffTurnRad > M_PI )
            diffTurnRad -= 2 * M_PI;
        ROS_DEBUG("Calculated turn: %f", diffTurnRad);
    
        double distance = std::sqrt( std::pow<double>(posX - target.first, 2) + std::pow<double>(posY - target.second, 2) );
        ROS_DEBUG("distance: %lf", distance);

        double speed = std::sqrt(distance);
        if ( std::abs(diffTurnRad) > 0.001 )
            speed *= 0.001 * std::exp(-diffTurnRad);
        
        ROS_DEBUG("speed: %lf", speed);

        if ( distance < 0.05 ) 
        {
            ++m_moveIdx;
            speed *= 0.1;
        }

        // return TwistWr(0,0, diffTurnRad);
        return TwistWr(speed, 0, diffTurnRad);
    }

private:
    // calculates rotation in radian between 0 - 2pi
    double CalcualateRotation(double x1, double y1, double x2, double y2)
    {
        double ydif = y2-y1;
        double xdif = x2-x1;

        double res = std::atan2(ydif, xdif);

        return res;
    }

    std::pair<float, float> GetNextTarget(float initialX, float initialY)
    {
        float x = initialX + m_moves[m_moveIdx].x * m_size;
        float y = initialY + m_moves[m_moveIdx].y * m_size;
        return std::make_pair(x, y);
    }

private:
    std::vector<RelativePos> m_moves;
    size_t m_moveIdx = 0;
    float m_size = 1;
};

void turtleWriteCallback(const std_msgs::String& msg)
{
}

float g_X, g_Y, g_W;
void turtlePoseCallback(const turtlesim::Pose& msg)
{
    // ROS_DEBUG("x y w: %f %f %f", msg.x, msg.y, msg.theta);
    g_X = msg.x;
    g_Y = msg.y;
    g_W = msg.theta;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ensargok");
    ros::NodeHandle n;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("turtle_writer", 1000, turtleWriteCallback);
    ros::Subscriber pose = n.subscribe("turtle1/pose", 1000, turtlePoseCallback);

    ros::Rate loop_rate(10);

    RelativePos pos1 = { 3, 1 };
    RelativePos pos2 = { 3, 3 };
    RelativePos posEnd = { 4, 1 };

    std::vector<RelativePos> pos = { pos1, pos2, posEnd };

    CharMove move = CharMove( std::move(pos), 0.5f );

    // geometry_msgs::Twist msgne = move.NextMove().msg;
    // ROS_WARN("Move: %f", msgne.angular.z);

    int count = 0;
    while(ros::ok())
    {
        // g_Writer.consume(cmd_vel, client);

        if ( count > 10 && !move.IsMovesDone() )
        {

            cmd_vel.publish( move.NextMove(5,5, g_X, g_Y, g_W) );
        }


        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
