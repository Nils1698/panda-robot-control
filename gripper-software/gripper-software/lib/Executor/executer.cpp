
#include "executer.h"

namespace exec {

Executer::Executer(String name, decltype(_mutex_list) mutex_list) : name(name), _mutex_list(mutex_list){
    is_active.store(false);
};

// Static executing functions (runs on seperate thread) //

void Executer::single_execution(void*ptr){
    Executer *executer = static_cast<Executer*>(ptr);
    
    for(Threads::Mutex& mutex : executer->_mutex_list){mutex.lock();}
    executer->fnc();
    for(Threads::Mutex& mutex : executer->_mutex_list){mutex.unlock();}

    executer->is_active.store(false);
}

void Executer::looped_execution(void*ptr){
    Executer *executer = static_cast<Executer*>(ptr);
    unsigned int last_fnc_call = 0;
    while(executer->is_ok()){
        if(millis() - last_fnc_call >= executer->loop_period){
            last_fnc_call = millis();

            for(Threads::Mutex& mutex : executer->_mutex_list){mutex.lock();}
            executer->fnc();
            for(Threads::Mutex& mutex : executer->_mutex_list){mutex.unlock();}
        }
        threads.yield();
    }
    if(executer->fnc_end){
        executer->fnc_end();
    }
    executer->is_active.store(false);
}

//////////////////////////////////////

void Executer::execute(std::function<void()> f, int each_ms, std::function<void()> f2, bool overtakable){
    if(is_executing()){
        if(accept_takeover){
            if(!stop(100)){
                warning_stream << name << " failed to take over execution!\n";
                return;
            }
        }else{
            warning_stream << name << " already executing!\n";
            return;
        }
    }

    is_active.store(true);

    stop_flag = false;
    fnc = f;
    fnc_end = f2;
    accept_takeover = overtakable;
    init_time = millis();

    if(each_ms >= 0){
        loop_period = each_ms;
        thread_id = threads.addThread(looped_execution, this);
        debug_stream << name << ": Loop thread added (id="<<thread_id<<")\n";
    }else{
        thread_id = threads.addThread(single_execution, this);
        debug_stream << name << ": Single thread added (id="<<thread_id<<")\n";
    }
}

// ALTERNATIVES:

// void Executer::execute(ThreadFunctionNone f){
//     if(is_executing()){
//         warning_stream << "Already executing!\n";
//         return;
//     }

//     if(thread_id!=-1 && threads.getState(thread_id)!=threads.ENDED){
//         threads.kill(thread_id);
//     }
//     stop_flag = false;
//     debug_stream << "Executor: Add thread\n";
//     thread_id = threads.addThread(f);
//     debug_stream << "Thread added\n";
// }

// In header:
// template<typename F>
// inline void execute(F&&lambda){

//     if(is_executing()){
//         warning_stream << "Already executing!\n";
//         return;
//     }

//     if(thread_id!=-1 && threads.getState(thread_id)!=threads.ENDED){
//         threads.kill(thread_id);
//     }
//     stop_flag = false;
//     debug_stream << "Executor: Add thread\n";
//     void* thread_arg = static_cast<void*>(&lambda);
//     thread_id = threads.addThread([](void*ptr){
//         F* f = static_cast<F*>(ptr);
//         (*f)(); // dereference and call
//     }, thread_arg);

//     debug_stream << "Thread added\n";
// }

/////////////////////

void Executer::kill_execution(){
    if(thread_id==-1 || !is_executing()){
        warning_stream << "No execution to kill\n";
        return;
    }
    threads.kill(thread_id);
    while (threads.getState(thread_id)!=threads.STOPPED){
        threads.yield();
    }
    is_active.store(false);
    thread_id=-1;
}

void Executer::request_stop(){
    Threads::Scope scope(interupt_lock);
    stop_flag = true;
}

bool Executer::stop(unsigned int timeout, bool kill){
    request_stop();
    unsigned long time_init = millis();
    while(is_executing()){
        if(millis()-time_init >= timeout){
            warning_stream << "Stopping thread timed out!\n";
            if(kill) kill_execution();
            return false;
        }
        threads.yield();
    }
    return true;
}

bool Executer::is_executing() const{
    return is_active.load();
    // return thread_id>0 && threads.getState(thread_id)==threads.RUNNING;
}


} // exec