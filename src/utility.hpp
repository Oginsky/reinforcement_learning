#ifndef UTILITY_HPP
#define UTILITY_HPP
#pragma once

#include <utility>
#include <fstream>
#include <map>


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



template <template <typename> class Functor, typename Value, typename T>
void write_if(Value& main_value, Value& value, T& main, T mem_main) {
    if(Functor<Value>{}(main_value, value)) {
        main_value = value;
        main = mem_main;
    }
}

#endif // UTILITY_HPP
