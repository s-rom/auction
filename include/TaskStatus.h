#ifndef AUCTION_TASKSTATUS
#define AUCTION_TASKSTATUS

namespace Auction
{
    enum class TaskStatus { WAITING, AUCTIONING, CONDUCTING, COMPLETED};
    class TaskStatusInfo;
}


#endif // AUCTION_TASKSTATUS