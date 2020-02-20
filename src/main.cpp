// #include "RobotManager.h"
#include "InfoReporter.h"
#include "SafeQueue.h"
#include "Message.h"


void test_messages(Auction::InfoReporter & info);

int main(int argc, char ** argv)
{
    using namespace Auction;

    InfoReporter info(Mode::COUT | Mode::TIME);
    // test_messages(info);

    info << "Hola que tal"<<" \n";

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
}

