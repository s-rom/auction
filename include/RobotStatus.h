
#ifndef ROBOT_STATUS
#define ROBOT_STATUS

#include <chrono>

namespace Auction
{
    enum RobotStatus {UNKOWN, ALIVE, DEAD};
    enum RobotRole {LEADING, HELPING, IDLE};
    struct RobotStatusInfo;
}


struct Auction::RobotStatusInfo
{
    RobotStatus current_status;
    RobotRole current_role;
    std::chrono::time_point<std::chrono::system_clock> last_time_point;
    bool first_time_point;

    static constexpr float TIME_LEAD_ALIVE_MILLIS = 3000;
    static constexpr float TOLERANCE = 200;

    RobotStatusInfo()
    :   
        first_time_point(true),
        current_status(RobotStatus::ALIVE),
        last_time_point(std::chrono::system_clock::now())
    {}

    void update_last_time_point()
    {
        this->last_time_point = std::chrono::system_clock::now();
    }

    float get_elapsed_millis()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now() - this->last_time_point).count();
    }


    

};

#endif