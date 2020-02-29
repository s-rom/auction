// #include "RobotManager.h"
#include "InfoReporter.h"
#include "SafeQueue.h"
#include "Message.h"
#include "RobotManager.h"
#include <vector>
#include <ctime>
#include <fstream>
#include <ros/ros.h>
#include <cstdlib>

void test_messages(Auction::InfoReporter & info);
void vector_test();
void work_capacity_test();
bool third_greater(const std::tuple<int,int,float> &a, 
                const std::tuple<int,int,float> &b);

void test_tuple();
void deadline_computation_test();
Auction::Task generate_task(int id);

class Prueba
{
public:
    template<class T>
    void es_std_string(const T & obj)
    {
        if (std::is_same<T, std::string>::value  || std::is_same<T,char*>::value)
        {
            
            std::cout << "Se ha convertido a string correctamente!";
        }
        else 
        {
            std::cout << "No es string\n";
        }
    }    

};


int main(int argc, char ** argv)
{
    // Auction::InfoReporter info("~/Desktop/prueba_info.txt");
    // info << "[Ambito] " << x << "\n";

   deadline_computation_test();



}

void test_tuple()
{

    using namespace Auction;
    srand(time(0));
    InfoReporter info(Mode::COUT | Mode::TIME);

    std::vector<std::tuple<int,int,float>> leader_bids;
    leader_bids.push_back(std::make_tuple(1,1,10));
    leader_bids.push_back(std::make_tuple(1,4,11));
    leader_bids.push_back(std::make_tuple(10,1,3));
    leader_bids.push_back(std::make_tuple(22,1000,4));
    
    auto max = *std::max_element(leader_bids.begin(),leader_bids.end(), third_greater);

    std::cout << "Max: " << std::get<0>(max) << "\n";
    std::cout << "Max: " << std::get<1>(max) << "\n";
    std::cout << "Max: " << std::get<2>(max) << "\n";
}

void deadline_computation_test()
{
    Auction::Task t = generate_task(0);
    
    std::cout << "Generated task\n";
    std::cout << t.to_string() << "\n";

    const int NUM_ROBOTS = 3;

    for (int i = 0; i<NUM_ROBOTS; i++)
    {
        float max_vel = (rand() % (13))  + 3; // 3 to 15 m/s
        float load_capacity = (rand() % 10) + 1; // 1 to 10 kg

        Auction::RobotManager r;
        r.set_max_linear_vel(max_vel);
        r.set_load_capacity(load_capacity);

        std::cout << "Generated robot \n\tvel: "<<max_vel<<"\n\tload_capacity: "<<load_capacity << "\n";
        std::cout << "---> WorkCapacity is: " << r.get_work_capacity(t) << "\n";
    }    
}



Auction::Task generate_task(int id)
{
    using namespace Auction;
    Point2D p(rand() % 20, rand() % 20); // 0 to 19 x y
    Point2D delivery(rand() % 20, rand() % 20); // 0 to 19 x y
    float workload = (rand() % 15) + 1; // 1 to 15 kg
    float dead_line = (rand() % 20000) + 50000; // 5s to 25s

    Task t(p,delivery, workload, dead_line, id);
    return t;
}



void work_capacity_test()
{
    using namespace Auction;
    Point2D p(rand() % 20, rand() % 20); // 0 to 19 x y
    Point2D delivery(rand() % 20, rand() % 20); // 0 to 19 x y
    int id = 0;
    float workload = (rand() % 15) + 1; // 1 to 15 kg
    float dead_line = (rand() % 20000) + 50000; // 5s to 25s

    Task t(p,delivery, workload, dead_line, ++id);
    std::cout << t.to_string() << std::endl;

    for (int time = 0; time < 1.5 * dead_line; time+=(dead_line)/10)
    {
        std::cout << "t: "<<time/1000<<", utility: "<<t.utility_function(time)<<"\n";
    }
}

bool third_greater(const std::tuple<int,int,float> &a, 
                const std::tuple<int,int,float> &b)
{
    return std::get<2>(a) < std::get<2>(b);
}

bool sort_by_bid(const std::pair<int,float> &a, 
                const std::pair<int,float> &b)
{
    return a.second > b.second;
}


void vector_test()
{
    std::vector<std::pair<int,float>> v;

    v.push_back(std::make_pair(1,0.2));
    v.push_back(std::make_pair(2,0.9));
    v.push_back(std::make_pair(3,0.1));
    v.push_back(std::make_pair(4,0.7));

    std::cout << "Unsorted set\n";
    for (auto it = v.begin(); it != v.end(); it++)
    {
        std::cout << it->first <<", "<< it->second << "\n";
    }

    std::sort(v.begin(),v.end(),sort_by_bid);
    std::cout << "Sorted set\n";
    for (auto it = v.begin(); it != v.end(); it++)
    {
        std::cout << it->first <<", "<< it->second << "\n";
    }
    
}


void test_messages(Auction::InfoReporter & info)
{
    Auction::LeaderOfTaskMessage m(1,2);
    // info << m.serialize() << "\n";

    Auction::LeaderOfTaskMessage m2(m.serialize());
    // info << m2.serialize() << "\n";

    if (m.serialize() == m2.serialize())
    {
        info << "Test passed for LeaderOfTaskMessage\n";
    }


    Auction::BidMessage bid(1,2,1,Auction::MessageType::BID_FOR_TASK);
    info << bid.serialize() + "\n";

    Auction::BidMessage bid2(bid.serialize());

    if (bid.serialize() == bid2.serialize())
    {
        info << "Test passed for BidMessage of type BID_FOR_TASK\n";
    }




}

