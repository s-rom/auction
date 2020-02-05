#ifndef SAFE_QUEUE
#define SAFE_QUEUE

#include <boost/thread/mutex.hpp>
#include <queue>


namespace Auction {

template<class T> 
class SafeQueue {

public:
    SafeQueue()
    {}

    void push(T t)
    {
        m.lock();
        q.push(t);
        m.unlock();
    }

    void front(T & t)
    {
        m.lock();
        t = q.front();
        m.unlock();
    }

    void pop(T & t)
    {
        m.lock();
        t = q.front();
        q.pop();
        m.unlock();
    }
    
    bool isEmpty()
    {
        m.lock();
        return q.empty();
        m.unlock();
    }
    
private:
    boost::mutex m; //< boost mutex for locking acces to the queue 
    std::queue<T> q; //< STL fifo queue
};
}


#endif

