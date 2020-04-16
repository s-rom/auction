#ifndef AUCTION_GROUP
#define AUCTION_GROUP


#include <vector>
#include <string>
#include "Task.h"


namespace Auction 
{
    struct Group;

    enum class ROBOT_ROLE { LEADING, HELPING, IDLE };
}


struct Auction::Group 
{

    Group(Task & t, int leader_id)
    :
        t(t),
        leader_id(leader_id)
    {}


    Auction::Task t;
    int leader_id;
    std::vector<int> helpers_id; 

    void add_helper(int id)
    {
        helpers_id.push_back(id);
    }


    std::string to_string()
    {
        std::string s = "--- Group ---\n";
        s += " Task: " + std::to_string(t.task_id) + "\n";
        s += " Leader: " + std::to_string(leader_id) + "\n";
        s += " Helpers: \n";
        
        for (const auto & helper : helpers_id)
        {
            s+="\t* Robot "+std::to_string(helper)+"\n";
        }

        return s;
    }

};




#endif