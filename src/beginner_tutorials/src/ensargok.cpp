#include "ros/console.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include <boost/math/policies/policy.hpp>
#include <cmath>
#include <utility>
#include <vector>
#include <math.h>

struct RelativePos
{
    float x, y;
    bool penDown = true; // for future use (failed to implement..)
};

geometry_msgs::Twist TwistWr( float x, float y, float w )
{
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;
    return msg;
}

class CharMoveBuilder;
class CharMove
{
public:
    CharMove(std::vector<RelativePos>&& moves, float size = 1.f)
        : m_moves(moves)
        , m_moveIdx(0)
        , m_size(size)
    { }

    using Builder = CharMoveBuilder;

    void Init(float initialX, float initialY)
    {
        m_initialX = initialX;
        m_initialY = initialY;
    }

    bool IsMovesDone()
    {
        return m_moves.size() == m_moveIdx;
    }

    geometry_msgs::Twist NextMove(float posX, float posY, float rotW)
    {
        if ( IsMovesDone() )
        {
            ROS_ERROR("There are not more move left!");
            return TwistWr(0, 0, 0);
        }

        ROS_DEBUG("rotW: %f", rotW);

        // calculate next position in map
        std::pair<float, float> target = GetNextTarget();

        ROS_DEBUG("position: %f, %f", posX, posY);
        ROS_DEBUG("target: %f, %f", target.first, target.second);

        // calculate angle towards target
        double nextTurnRad = CalcualateRotation(posX, posY, target.first, target.second);
        ROS_DEBUG("nextTurnRad: %lf", nextTurnRad);

        // diffrence between our angle and target angle
        double diffTurnRad = nextTurnRad - rotW;
        ROS_DEBUG("diffTurnRad = nextTurnRad - prewRot : %lf", diffTurnRad);

        if ( diffTurnRad > M_PI )
            ROS_INFO("diffTurnRad > M_PI");

        if ( diffTurnRad > M_PI )
            diffTurnRad -= 2 * M_PI;
        ROS_DEBUG("Calculated turn: %f", diffTurnRad);
    
        // disance between locations (a^2+b^2)^.5
        double distance = std::sqrt( std::pow<double>(posX - target.first, 2) + std::pow<double>(posY - target.second, 2) );
        ROS_DEBUG("distance: %lf", distance);

        // if we have to turn around, we should not move fast
        double speed = std::sqrt(distance);
        if ( std::abs(diffTurnRad) > 0.001 )
            speed *= 0.001 * std::exp(-diffTurnRad);
        
        ROS_DEBUG("speed: %lf", speed);

        // Are we close enough?
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

    std::pair<float, float> GetNextTarget()
    {
        float x = m_initialX + m_moves[m_moveIdx].x * m_size;
        float y = m_initialY + m_moves[m_moveIdx].y * m_size;
        return std::make_pair(x, y);
    }

private:
    std::vector<RelativePos> m_moves;
    size_t m_moveIdx = 0;
    float m_size = 1;
    float m_initialX=-1;
    float m_initialY=-1;
};

class CharMoveBuilder
{
public:
    CharMoveBuilder() = default;

    CharMoveBuilder& next(float x, float y, bool penDown = true)
    {
        m_moves.push_back({x, y, penDown});
        return *this;
    }

    CharMove build(float size = 1.f)
    {
        return CharMove(std::move(m_moves), size);
    }

private:
    std::vector<RelativePos> m_moves = {};
};

void turtleWriteCallback(const std_msgs::String& msg)
{
}

float g_X, g_Y, g_W;
void turtlePoseCallback(const turtlesim::Pose& msg)
{
    ROS_DEBUG("x y w: %f %f %f", msg.x, msg.y, msg.theta);
    g_X = msg.x;
    g_Y = msg.y;
    g_W = msg.theta;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ensarwriter");
    ros::NodeHandle n;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber pose = n.subscribe("turtle1/pose", 1000, turtlePoseCallback);

    ros::Rate loop_rate(10);

    RelativePos pos1 = { 3, 1 };
    RelativePos pos2 = { 3, 3 };
    RelativePos posEnd = { 4, 1 };

    std::vector<RelativePos> pos = { pos1, pos2, posEnd };

    CharMove moveA = CharMove::Builder().next(1.f*0.75, 2).next(1.5f*0.75, 1).next(0.5f*0.75, 1).next(1.5f*0.75, 1).next(2*0.75, 0).build(1.5f);
    CharMove moveB = CharMove::Builder().next(0, 2).next(0.25f, 2).next(0.5f, 1.5f).next(0.25,1.f).next(0, 1.f).next(0.25,1.f).next(0.5f, 0.5f).next(0.25f, 0).next(0,0).next(1.f, 0).build();
    CharMove moveC = CharMove::Builder().next(0, 1.5f, false).next(0.5, 2).next(1,2).next(0.5,2).next(0, 1.5).next(0, .5f).next(.5f, 0).next(1, 0).build();
    CharMove moveD = CharMove::Builder().next(0, 2).next(.75, 1.5).next(.75, .5).next(0,0).next(1.5 ,0, false).build();
    CharMove moveE = CharMove::Builder().next(0,2).next(1,2).next(0,2).next(0,1).next(1,1).next(0,1).next(0,0).next(1, 0).next(1.5, 0, false).build();
    CharMove moveF = CharMove::Builder().next(0,2).next(1,2).next(0,2).next(0,1).next(1,1).next(0,1).next(0,0).next(1.5,0, false).build();
    CharMove move0 = CharMove::Builder().next(0,.5,false).next(0,1.5).next(.5*0.5,2).next(1.5*0.5,2).next(2*0.5,1.5).next(2*0.5,.5).next(1.5*0.5,0).next(0.5*0.5,0).next(0,0.5).next(1.75,0,false).build();
    CharMove move1 = CharMove::Builder().next(0,1.5,false).next(1,2).next(1,0).next(1.5,0).build();
    CharMove move2 = CharMove::Builder().next(0,1.5,false).next(0.5,2).next(1.5,2).next(0,0).next(1.5,0).next(2,0,false).build();
    CharMove move3 = CharMove::Builder().next(0,1.5,false).next(0,1.75).next(0.25,2).next(1,2).next(1,1.25).next(0.75,1).next(0.5,1).next(0.75,1).next(1,0.75).next(1,0).next(0.25,0).next(0,0.25).next(1.5,0,false).build();
    CharMove move4 = CharMove::Builder().next(1.25,0,false).next(1.25,2).next(0,0.25).next(1.5,0.25).next(1.75,0,false).build();
    CharMove move5 = CharMove::Builder().next(1,2,false).next(0,2).next(0,1).next(0.75,1).next(1,0.75).next(1,.25).next(0.75,0).next(0,0).next(1.25,0,false).build();
    CharMove move6 = CharMove::Builder().next(0.75,2,false).next(0.25,2).next(0,1.75).next(0,0.25).next(0.25,0).next(0.75,0).next(1,0.25).next(1,0.75).next(0.75,1).next(0,1).next(1.25,0,false).build();
    CharMove move7 = CharMove::Builder().next(0,2,false).next(1,2).next(0,0).next(1.25,0,false).build();
    CharMove move8 = CharMove::Builder().next(0.75,0,false).next(1,0.25).next(1,.75).next(0.75,1).next(.25,1).next(0,1.25).next(0,1.75).next(0.25,2).next(.75,2).next(1,1.75).next(1,1.25).next(0.75,1).next(.25,1).next(0,.75).next(0,.25).next(.25,0).next(0.75,0).next(1.25,0,false).build();
    CharMove move9 = CharMove::Builder().next(0,.25,false).next(.25,0).next(.75,0).next(1,.25).next(1,1.75).next(0.75,2).next(.25,2).next(0,1.75).next(0,1.25).next(.25,1).next(1,1).next(1.25,0,false).build();
    CharMove move = CharMove::Builder().next(1,1).next(0,1,false).next(1,0).next(1.25,0,false).build();

    int count = 0;
    while(ros::ok())
    {
        // g_Writer.consume(cmd_vel, client);

        if ( count == 10 )
            move.Init(g_X, g_Y);
        if ( count > 10 && !move.IsMovesDone() )
        {
            cmd_vel.publish( move.NextMove(g_X, g_Y, g_W) );
        }


        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
