#include "MessageSystem.h"

int main(int argc, char ** argv)
{
    using namespace Auction;
    MessageSystem ms;
    NetProfile n("localhost", "25555");
    LeaderRequestMessage lreq(1,2,2,2.0f);

    ms.send_message(lreq,n);

}