#include <stdio.h>
#include <iostream>
#include "RobotManager.h"


int main()
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    Task t(Point2D(),Point2D(),1,2,3);
    NewTaskMessage nt(t);
    MessageSystem ms;

    Message * m = ms.create_message_from("#0#1#1#1#1#1#2#3#");
    if (m->type == MessageType::NEW_TASK)
    {
        NewTaskMessage * nt = dynamic_cast<NewTaskMessage*>(m);
        cout << nt->serialize() << endl;
    }
    else
    {
        cout << "No es new task" << endl;
    }

}