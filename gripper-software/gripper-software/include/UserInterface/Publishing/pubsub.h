
#pragma once

#include "Arduino.h"
#include <functional>
#include <map>
#include <vector>
#include <algorithm>
#include "Streaming.h"

#include "ForceController.h"

#include "fingers.h"
#include <functional>

#include "Motor.h"

#include "TactilePerception.h"

#include "HW_prototype.h" // (hw::tactile_mutex)

namespace PUB {

struct publisher_t {
    String id;
    std::map<const String, std::function<void(Print&, uint8_t)>> categories;
    std::map<const String, uint8_t> spec_map;
    void printTo(Print &out){
        for(auto cat : categories){
            out << cat.first << " ";
        }
    }
};

struct subscription {
    static const uint8_t NO_SPEC = 255;
    std::pair<const String, std::function<void(Print&, uint8_t)>> &sub_type;
    uint8_t sub_spec;
    subscription(std::pair<const String, std::function<void(Print&, uint8_t)>> &sub_type, uint8_t sub_spec) : sub_type(sub_type), sub_spec(sub_spec){}
    void printTo(Print& out){
        out << sub_type.first;
        if(sub_spec!=NO_SPEC)
            out << " ("<<sub_spec<<")";
    }
    void publish(Print& out){
        sub_type.second(out, sub_spec);
    }
};

class pubsub {
private:
    std::vector<publisher_t> options;
    std::vector<subscription> subscriptions;
public:
    pubsub(std::vector<publisher_t> options) : options(options){}

    bool subscribe(String id, String cat_id, uint8_t spec){
        // Find option with matching id and cat_id
        auto pub_it = std::find_if(options.begin(), options.end(), [&](const publisher_t& p){return p.id==id;});
        if(pub_it!=options.end())
        {
            auto cat = pub_it->categories.find(cat_id);
            if(cat != pub_it->categories.end()){
                subscriptions.push_back(subscription(*cat, spec));
                return true;
            }
        }
        return false;
    }

    bool subscribe(String id, String cat_id, String spec_str){
        // Find option with matching id and cat_id
        auto pub_it = std::find_if(options.begin(), options.end(), [&](const publisher_t& p){return p.id==id;});
        if(pub_it!=options.end())
        {
            auto cat = pub_it->categories.find(cat_id);
            if(cat != pub_it->categories.end()){
                auto spec_it = pub_it->spec_map.find(spec_str);
                if(spec_it != pub_it->spec_map.end()){
                    uint8_t spec = spec_it->second;
                    subscriptions.push_back(subscription(*cat, spec));
                }
                return true;
            }
        }
        return false;
    }    

    bool subscribe(String id, String cat_id){
        return subscribe(id, cat_id, 0);
    }

    void unsubscribe_all(){
        subscriptions.clear();
    }

    void publish_subscriptions(Print& out){
        for(subscription &sub : subscriptions){
            sub.publish(out);
        }
        if(subscriptions.size()>0) out<<"\n";
    }

    void printSubscription(Print &out){
        for(subscription &sub : subscriptions){
            sub.printTo(out);
            out << "\n";
        }
    }

    void printOptions(Print &out){
        out << "type: category1 category2 ...\n";
        for(auto opt : options){
            out << opt.id << ": ";
            opt.printTo(out);
            out << "\n";
        }
    }
};

} // PUB