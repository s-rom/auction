#ifndef INFO_REPORTER
#define INFO_REPORTER

#include <fstream>
#include <string>
#include <iostream>
#include <cmath>
#include <bitset>
#include <ros/ros.h>

namespace Auction
{
    enum Mode {FILE=1, COUT=2, ROS=4};
    class InfoReporter;
}

class Auction::InfoReporter
{
public:
    
    InfoReporter()
    {
        this->options = Mode::COUT;
    }
    
    InfoReporter(int mode)
    {
        this->options = mode;
    }

    InfoReporter(std::string name, int mode)
    :
        f(name, std::ios::out)
    {
        this->options = mode;
    }

    InfoReporter(std::string name)
    :
        f(name, std::ios::out)
    {
        this->options = Mode::FILE | Mode::COUT;
    }

    ~InfoReporter()
    {
        close();
    }


    template<class T>
    InfoReporter& operator<<(const T& obj)
    {
        
        if (isFileMode() && f.is_open())
        {
            f << obj;
        }

        if (isCoutMode())
        {
            std::cout << obj;
        }

        if (isROSMode())
        {
            ROS_INFO_STREAM(obj);
        }
        
        return *this;
    }


    void check_options()
    {
        std::cout << "Options: "<<this->options<<std::endl;
        std::cout << "FileMode: "<<isFileMode() << std::endl;
        std::cout << "RosMode: "<<isROSMode() << std::endl;
        std::cout << "CoutMode: "<<isCoutMode() << std::endl;
    }

    void setOptions(Mode options_param)
    {
        this->options = options_param;
    }


    void close()
    {
        f.close();
    }

private:

    std::fstream f;
    std::bitset<3> options;



    bool isFileMode()
    {
        return options[log2(FILE)];
    }

    bool isCoutMode()
    {
        return options[log2(COUT)];
    }

    bool isROSMode()
    {
        return options[log2(ROS)];
    }

};

#endif