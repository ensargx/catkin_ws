#include "beginner_tutorials/TurtleWriteRequest.h"
#include "beginner_tutorials/TurtleWriteResponse.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/SetPenRequest.h"
#include <boost/math/policies/policy.hpp>
#include <cmath>
#include "beginner_tutorials/TurtleWrite.h"
#include <list>
#include <utility>
#include <vector>
#include <math.h>

struct RelativePos
{
    float x, y;
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

    std::pair<geometry_msgs::Twist, bool> NextMove(float posX, float posY, float rotW)
    {
        if ( IsMovesDone() )
        {
            ROS_ERROR("There are not more move left!");
            return std::make_pair(TwistWr(0, 0, 0), true);
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
        return std::make_pair(TwistWr(speed, 0, diffTurnRad), IsPenDown());
    }

    void setSize(float size)
    {
        m_size = size;
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

    bool IsPenDown() 
    {
        return m_moves[m_moveIdx].penDown;
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

class CharMoveService
{
public:
    CharMoveService()
    {
    }

    CharMove get(int idx) { return moves[idx]; }
    /* const& ile gereksiz kopyalardan kaçınılabilir ama
     * bunun için moveIdx'ler ile RelavtivePos'lar ayrılmalı */
    CharMove get0() { return moves[0]; }
    CharMove get1() { return moves[1]; }
    CharMove get2() { return moves[2]; }
    CharMove get3() { return moves[3]; }
    CharMove get4() { return moves[4]; }
    CharMove get5() { return moves[5]; }
    CharMove get6() { return moves[6]; }
    CharMove get7() { return moves[7]; }
    CharMove get8() { return moves[8]; }
    CharMove get9() { return moves[9]; }
    CharMove getA() { return moves[10]; }
    CharMove getB() { return moves[11]; }
    CharMove getC() { return moves[12]; }
    CharMove getD() { return moves[13]; }
    CharMove getE() { return moves[14]; }
    CharMove getF() { return moves[15]; }

private:
    std::array<CharMove, 16> moves = {
        CharMove::Builder().next(0,.5,false).next(0,1.5).next(.5*0.5,2).next(1.5*0.5,2).next(2*0.5,1.5).next(2*0.5,.5).next(1.5*0.5,0).next(0.5*0.5,0).next(0,0.5).next(1.75,0,false).build(),
        CharMove::Builder().next(0,1.5,false).next(1,2).next(1,0).next(1.5,0, false).build(),
        CharMove::Builder().next(0,1.5,false).next(0.5,2).next(1.5,2).next(0,0).next(1.5,0).next(2,0,false).build(),
        CharMove::Builder().next(0,1.5,false).next(0,1.75).next(0.25,2).next(1,2).next(1,1.25).next(0.75,1).next(0.5,1).next(0.75,1).next(1,0.75).next(1,0).next(0.25,0).next(0,0.25).next(1.5,0,false).build(),
        CharMove::Builder().next(1.25,0,false).next(1.25,2).next(0,0.25).next(1.5,0.25).next(1.75,0,false).build(),
        CharMove::Builder().next(1,2,false).next(0,2).next(0,1).next(0.75,1).next(1,0.75).next(1,.25).next(0.75,0).next(0,0).next(1.25,0,false).build(),
        CharMove::Builder().next(0.75,2,false).next(0.25,2).next(0,1.75).next(0,0.25).next(0.25,0).next(0.75,0).next(1,0.25).next(1,0.75).next(0.75,1).next(0,1).next(1.25,0,false).build(),
        CharMove::Builder().next(0,2,false).next(1,2).next(0,0).next(1.25,0,false).build(),
        CharMove::Builder().next(0.75,0,false).next(1,0.25).next(1,.75).next(0.75,1).next(.25,1).next(0,1.25).next(0,1.75).next(0.25,2).next(.75,2).next(1,1.75).next(1,1.25).next(0.75,1).next(.25,1).next(0,.75).next(0,.25).next(.25,0).next(0.75,0).next(1.25,0,false).build(),
        CharMove::Builder().next(0,.25,false).next(.25,0).next(.75,0).next(1,.25).next(1,1.75).next(0.75,2).next(.25,2).next(0,1.75).next(0,1.25).next(.25,1).next(1,1).next(1.25,0,false).build(),
        CharMove::Builder().next(1.f*0.75, 2).next(1.5f*0.75, 1).next(0.5f*0.75, 1).next(1.5f*0.75, 1).next(2*0.75, 0).build(1.5f),
        CharMove::Builder().next(0, 2).next(0.25f, 2).next(0.5f, 1.5f).next(0.25,1.f).next(0, 1.f).next(0.25,1.f).next(0.5f, 0.5f).next(0.25f, 0).next(0,0).next(1.f, 0).build(),
        CharMove::Builder().next(0, 1.5f, false).next(0.5, 2).next(1,2).next(0.5,2).next(0, 1.5).next(0, .5f).next(.5f, 0).next(1, 0).build(),
        CharMove::Builder().next(0, 2).next(.75, 1.5).next(.75, .5).next(0,0).next(1.5 ,0, false).build(),
        CharMove::Builder().next(0,2).next(1,2).next(0,2).next(0,1).next(1,1).next(0,1).next(0,0).next(1, 0).next(1.5, 0, false).build(),
        CharMove::Builder().next(0,2).next(1,2).next(0,2).next(0,1).next(1,1).next(0,1).next(0,0).next(1.5,0, false).build(),
    };
    // CharMove moveX = CharMove::Builder().next(1,1).next(0,1,false).next(1,0).next(1.25,0,false).build();
};

class TurtleService : public ros::NodeHandle
{
public:
    TurtleService(std::string&& turtleName)
        : ros::NodeHandle()
        , m_TurtlePose( subscribe("/turtle1/pose", 1000, &TurtleService::turtlePoseCallback, this) )
        , m_SetPenClient( serviceClient<turtlesim::SetPen>("turtle1/set_pen") )
        , m_CmdVel( advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000) )
        , m_Server( advertiseService("turtle_write", &TurtleService::handleWriteRequest, this) )
        , m_TurtleName( turtleName )
        , m_CharMoveService( CharMoveService() )
    {
    }

    void loop() 
    {
        ROS_INFO("Starting loop for turtle_writer_server");
        while( ros::ok() )
        {
            if ( !m_moves.empty() )
            {
                CharMove& move = m_moves.front();
                if ( m_moving == false )
                {
                    move.Init(m_X, m_Y);
                    m_moving = true;
                }
                if ( move.IsMovesDone() )
                {
                    m_moves.pop_front();
                    m_moving = false;
                    m_penDown = true;
                    turtleSetPen();
                }
                else 
                {
                    auto msg = move.NextMove(m_X, m_Y, m_W);
                    m_CmdVel.publish( msg.first );
                    if ( msg.second != m_penDown ) {
                        m_penDown = msg.second;
                        turtleSetPen();
                    }
                }
            }
            ros::spinOnce();
            m_loopRate.sleep();
        }
    }

private:
    void turtleSetPen()
    {
        turtlesim::SetPen req = {};
        req.request.off = !m_penDown;
        req.request.r = 200;
        req.request.g = 200;
        req.request.b = 200;
        req.request.width = 3;
        m_SetPenClient.call(req);
    }

    void turtlePoseCallback(const turtlesim::Pose& msg)
    {
        ROS_DEBUG("x y w: %f %f %f", msg.x, msg.y, msg.theta);
        m_X = msg.x;
        m_Y = msg.y;
        m_W = msg.theta;
    }

    int charToHexValue(char c) {
        if (c >= '0' && c <= '9')
            return c - '0';                // 0–9 → 0–9
        else if (c >= 'a' && c <= 'f')
            return 10 + (c - 'a');         // a–f → 10–15
        else if (c >= 'A' && c <= 'F')
            return 10 + (c - 'A');         // A–F → 10–15
        else
            return -1; // geçersiz karakter
    }

    bool handleWriteRequest(beginner_tutorials::TurtleWriteRequest& req, beginner_tutorials::TurtleWriteResponse& res)
    {
        ROS_INFO("gelen mesaj: %s | scale: %f", req.message.c_str(), req.scale);

        for (char c : req.message)
        {
            int index = charToHexValue(c);
            if ( index != -1 )
            {
                CharMove move = m_CharMoveService.get(index);
                move.setSize(req.scale);
                m_moves.push_back(move);
            } 
            else
            {
                ROS_WARN("Incorrect hex message!");
                return false;
            }
        }

        return true;
    }

private:
    std::list<CharMove> m_moves = {};
    ros::Subscriber m_TurtlePose;
    ros::ServiceClient m_SetPenClient;
    ros::Publisher m_CmdVel;
    ros::ServiceServer m_Server;
    std::string m_TurtleName;
    CharMoveService m_CharMoveService;
    bool m_penDown = true; // assume this is true for now.
    bool m_moving = false;
    ros::Rate m_loopRate = 10;
    float m_X, m_Y, m_W;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_write_server");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    TurtleService service ( std::string("turtle1") );

    service.loop();

    return 0;
}
