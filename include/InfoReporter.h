#ifndef INFO_REPORTER
#define INFO_REPORTER

#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <bitset>
#include <chrono>

namespace Auction
{
    enum Mode {FILE=1, COUT=2, ROS=4, TIME=8};
    class InfoReporter;
}

class Auction::InfoReporter
{
public:
    
    InfoReporter()
    {
        this->options = Mode::COUT | Mode::TIME;
    }
    
    InfoReporter(int mode)
    {
        this->options = mode;
    }

    InfoReporter(std::string name, int mode)
    :
        f(name, std::ios::out)
    {
        this->options = mode | Mode::FILE;
    }

    InfoReporter(std::string name)
    :
        f(name, std::fstream::out | std::fstream::trunc)
    {
        this->options = Mode::FILE | Mode::COUT;
    }
    
    ~InfoReporter()
    {
        close();
    }


    std::string make_string(const int obj)
    {
        return std::to_string(obj);
    }

    std::string make_string(const float obj)
    {
        return std::to_string(obj);
    }

    std::string make_string(const char * obj)
    {
        return std::string(obj);
    }

    std::string make_string(const std::string s)
    {
        return s;
    }

    template<class T>
    InfoReporter& operator<<(const T& obj)
    {
        using std::chrono::system_clock;
        auto now_t = system_clock::to_time_t(system_clock::now());

        bool put_time = false;
        std::string s = make_string(obj);
        put_time = s.find('[') != std::string::npos;

        if (isFileMode() && f.is_open())
        {
            if (isTimeMode() && put_time) f << std::put_time(std::localtime(&now_t),"<%T> ");
            f << obj;        

        }

        if (isCoutMode())
        {
            if(isTimeMode() && put_time) std::cout << std::put_time(std::localtime(&now_t),"<%T> ");
            std::cout << obj;

        }

        if (isROSMode())
        {
            // ROS_INFO_STREAM(obj);
        }
        
        return *this;
    }


    void check_options()
    {
        std::cout << "Options: "<<this->options<<std::endl;
        std::cout << "FileMode: "<<isFileMode() << std::endl;
        std::cout << "RosMode: "<<isROSMode() << std::endl;
        std::cout << "CoutMode: "<<isCoutMode() << std::endl;
        std::cout << "TimeMode: "<<isTimeMode() << std::endl;
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
    std::bitset<4> options;

    bool isTimeMode()
    {
        return options[log2(TIME)];
    }

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