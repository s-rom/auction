// #include "RobotManager.h"
#include "InfoReporter.h"
#include "SafeQueue.h"
#include "Message.h"
#include <vector>
#include <ctime>
#include <cstdlib>

void test_messages(Auction::InfoReporter & info);
void vector_test();
void work_capacity_test();

int main(int argc, char ** argv)
{
    using namespace Auction;
    srand(time(0));
    InfoReporter info(Mode::COUT | Mode::TIME);

    work_capacity_test();
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

