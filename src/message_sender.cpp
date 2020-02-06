#include "MessageSystem.h"
#include <unordered_map>
#include <string>

int main(int argc, char ** argv)
{
    using namespace Auction;
    MessageSystem ms;
    NetProfile n("localhost", "25555");
    LeaderRequestMessage lreq(1,2,2,2.0f);


    int my_port = 25555;
    std::unordered_map<int,NetProfile> net_list;
    for (int i = 0; i<3; i++)
    {
        int port = my_port + i;
        std::string port_str = std::to_string(port);
        net_list[i] = NetProfile("localhost", port_str);
    }

    ms.broadcast_message(lreq, net_list);

}