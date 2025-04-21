#include "ros/console.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "turtlesim/SetPen.h"
#include <cmath>
#include <vector>
#include <list>
#include <math.h>

struct RelativePos
{
    int x, y;
    bool penDown = true;
};

struct TurtleMove
{
    geometry_msgs::Twist msg;
    bool penDown = true;
};

class CharMove
{
public:
    CharMove(std::vector<RelativePos>&& moves)
        : m_moves(moves)
        , m_moveIdx(0)
        , m_turning(true)
        , m_prevRot(0)
    { }

    bool IsMovesDone()
    {
        return m_moves.size() == m_moveIdx && m_turning;
    }

    TurtleMove NextMove()
    {
        if ( IsMovesDone() )
        {
            ROS_ERROR("This should never happen");
            return {};
        }

        // if we should turn toward the location we'll go
        if ( m_turning )
        {
            ROS_DEBUG("We'll ROTATE!");
            const RelativePos& prevPos = PreviousPos();
            const RelativePos& nextPos = NextPos();

            double nextTurnRad = CalcualateRotation(prevPos.x, prevPos.y, nextPos.x, nextPos.y);
            ROS_DEBUG("nextPos: %d, %d", nextPos.x, nextPos.y);
            ROS_DEBUG("prevPos: %d, %d", prevPos.x, prevPos.y);
            ROS_DEBUG("nextTurnRad: %lf", nextTurnRad);

            double diffTurnRad = nextTurnRad - m_prevRot;
            ROS_DEBUG("diffTurnRad = nextTurnRad - prewRot : %lf", diffTurnRad);
        
            m_turning = false;
            m_prevRot = nextTurnRad;

            geometry_msgs::Twist msg = {};
            msg.angular.z = diffTurnRad;

            return {msg};
        }
        else // We should go forward.
        {
            ROS_DEBUG("We'll MOVE!");
            const auto& prevPos = PreviousPos();
            const auto& nextPos = NextPos();

            ROS_DEBUG("nextPos: %d, %d", nextPos.x, nextPos.y);
            ROS_DEBUG("prevPos: %d, %d", prevPos.x, prevPos.y);
            ROS_DEBUG("calculate: sqrt(%lf + %lf)", std::pow<double>(prevPos.x - nextPos.x, 2), std::pow<double>(prevPos.y - nextPos.y, 2) );
            double distance = std::sqrt( std::pow<double>(prevPos.x - nextPos.x, 2) + std::pow<double>(prevPos.y - nextPos.y, 2) );
            ROS_DEBUG("distance: %lf", distance);

            geometry_msgs::Twist msg = {};
            
            ++m_moveIdx;
            m_turning = true;

            msg.linear.x = distance;

            return { msg };
        }
    }

private:
    // calculates rotation in radian between 0 - 2pi
    double CalcualateRotation(double x1, double y1, double x2, double y2)
    {
        double ydif = y2-y1;
        double xdif = x2-x1;

        double res = std::atan2(ydif, xdif);

        if ( res < 0 )
            res += 2 * M_PI;

        return res;
    }

    const RelativePos& PreviousPos()
    {
        static RelativePos empty = {};
        if ( m_moveIdx == 0 )
            return empty;
        return m_moves[m_moveIdx-1];
    }

    const RelativePos& NextPos()
    {
        return m_moves[m_moveIdx];
    }

private:
    std::vector<RelativePos> m_moves;
    size_t m_moveIdx = 0;
    bool m_turning = true;
    double m_prevRot = 0;
};

/*
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
        srv.request.width = 1.f;
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
*/

void turtleWriteCallback(const std_msgs::String& msg)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ensargok");
    ros::NodeHandle n;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::ServiceClient client = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    ros::Subscriber sub = n.subscribe("turtle_writer", 1000, turtleWriteCallback);

    ros::Rate loop_rate(1);

    RelativePos pos1 = { 3, 1 };
    RelativePos pos2 = { 3, 5 };
    RelativePos pos3 = { 0, 0 };
    RelativePos pos4 = { 1, 1 };
    RelativePos pos5 = { 3, 5 };

    std::vector<RelativePos> pos = { pos1, pos2, pos3, pos4, pos5 };

    CharMove move = ( std::move(pos) );

    // geometry_msgs::Twist msgne = move.NextMove().msg;
    // ROS_WARN("Move: %f", msgne.angular.z);

    int count = 0;
    while(ros::ok())
    {
        // g_Writer.consume(cmd_vel, client);

        if ( count > 1 && !move.IsMovesDone() )
        {
            cmd_vel.publish( move.NextMove().msg );
        }


        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
