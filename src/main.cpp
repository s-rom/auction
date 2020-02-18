// #include "RobotManager.h"
#include "InfoReporter.h"


int main(int argc, char ** argv)
{
    using Auction::InfoReporter;
    using Auction::Mode;
    
    InfoReporter info("/home/sergi/Desktop/hola.txt", Mode::COUT | Mode::FILE);

    int x = 7;
    char msg[6];
    strcpy(msg,"Hola");

    info << x << "," << msg << "\n";

    // info.check_options();
    info.close();
}



