#include <iostream>
#include "NetProfile.h"
#include "Task.h"
#include "Point2D.h"
#include "Message.h"
#include <cstring>

#include <sstream>
#include <algorithm>

#include "MessageSystem.h"

int main()
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    
    MessageSystem ms;


    char msg[30];
    strcpy(msg,"#2#2#2#");
    
    Message * m = ms.createMessageFrom(msg);
    delete m;
}


