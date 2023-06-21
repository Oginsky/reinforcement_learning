#ifndef UTILITY_HPP
#define UTILITY_HPP
#pragma once

#include <utility>
#include <fstream>
#include <map>
#include <random>


template <typename Container>
bool write_in_file(std::string filename, Container& container) {
    std::ofstream fout(filename);
    if(!fout.is_open()) return false;

    for(auto& item : container)
        fout << item << " ";

    return true;
} 

template <typename K, typename V>
bool write_in_file(std::string filename, std::map<K, V>& container) {
    std::ofstream fout(filename);
    if(!fout.is_open()) return false;

    for(auto& item : container)
        fout << item.second << " ";

    return true;
}

template <typename V>
bool write_in_file(std::string filename, std::map<std::pair<int, int>, V>& container) {
    std::ofstream fout(filename);
    if(!fout.is_open()) return false;

    for(auto& item : container)
        fout << item.first.first << " " << item.first.second << " " << item.second << "\n";

    return true;
}



template <template <typename> class Functor, typename Value, typename T>
void write_if(Value& main_value, Value& value, T& main, T mem_main) {
    if(Functor<Value>{}(main_value, value)) {
        main_value = value;
        main = mem_main;
    }
}


bool rand(double prob) {
    static std::mt19937_64 engine{std::random_device{}()};
    auto max = engine.max();

    double threshold_value = max * prob;
    return engine() < threshold_value;
}


template <typename state_t, typename action_t>
void converse_value_action(const std::map<state_t, std::map<action_t, double>>& map_action,
                           std::map<state_t, double>& map_state)
{
    for(const auto& entry: map_action) {
        auto state = entry.first;
        auto& inner_map = entry.second;
        double max_value = inner_map.begin()->second;
        for(const auto& val: inner_map)
            max_value = std::max(max_value, val.second);

        map_state[state] = max_value;
    }
}

#endif // UTILITY_HPP
