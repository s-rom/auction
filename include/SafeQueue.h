#ifndef SAFE_QUEUE
#define SAFE_QUEUE

#include <boost/thread/mutex.hpp>
#include <deque>


namespace Auction {

template<class T> 
class SafeQueue {

public:

    SafeQueue()
    {}

    void push(T t)
    {
        m.lock();
        q.push_back(t);
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
        q.pop_front();
        m.unlock();
    }
    
    typename std::deque<T>::iterator begin()
    {
        typename std::deque<T>::iterator it;
        m.lock();
        it = q.begin();
        m.unlock();
        return it;
    }

    typename std::deque<T>::iterator end()
    {
        typename std::deque<T>::iterator it;
        m.lock();
        it = q.end();
        m.unlock();
        return it;
    }

    void erase(typename std::deque<T>::iterator it)
    {
        m.lock();
        q.erase(it);
        m.unlock();
    }

    bool isEmpty()
    {
        bool ret;
        m.lock();
        ret = q.empty();
        m.unlock();
        return ret;
    }
    
private:
    boost::mutex m; //< boost mutex for locking access to the queue 
    std::deque<T> q; //< STL deque
};
}


#endif

