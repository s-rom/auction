#include <stdio.h>
#include <iostream>
#include "RobotManager.h"


int main()
{
    using std::cout;
    using std::endl;
    using namespace Auction;

    NetProfile n1("localhost", "25555");
    NetProfile n2("localhost", "25555");
    NetProfile n3("localhost", "25556");
    
    cout << (n1 == n2) << endl;
    cout << (n1 == n3) << endl;

}