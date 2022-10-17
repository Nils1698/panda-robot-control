#pragma once

#include "TeensyThreads.h"
#include "UserStream.h"

#include "functional"
#include <atomic>
#include <vector>

namespace exec {

class Executer
{
private:
    int thread_id=-1;
    Threads::Mutex interupt_lock;
    bool stop_flag = false;
    std::atomic<bool> is_active;
    const String name;

    std::function<void()> fnc, fnc_end;
    unsigned int loop_period;
    unsigned int init_time;

    bool accept_takeover=false;

    std::vector<std::reference_wrapper<Threads::Mutex>> _mutex_list;

	static void looped_execution(void *ptr);
    static void single_execution(void *ptr);

public:
    Executer(String name, decltype(_mutex_list) mutex_list);
    Executer(String name)  : Executer(name, {}){}
    Executer() : Executer("Executor") {}


    void execute(std::function<void()> f, int each_ms = -1, std::function<void()> f2 = nullptr, bool overtakable=false);
    inline void execute_loop(std::function<void()> f, unsigned int T, std::function<void()> f2 = nullptr){execute(f,T,f2);}
    inline void execute_receding(std::function<void()> f, unsigned int T, std::function<void()> f2 = nullptr){execute(f,T,f2,true);}

    void request_stop();
    bool stop(unsigned int timeout, bool kill=false);
    void kill_execution();
    bool is_executing() const;
    inline unsigned long time_alive(){
        return millis()-init_time;
    }

    inline bool is_ok(){
        Threads::Scope scope(interupt_lock);
        return stop_flag==false;
    }

    inline int getID(){
        return thread_id;
    }

    inline void printTo(Print &out){
        out << name << (is_executing() ? " is running" : " not running") << "\n";
    }

};

} // exec