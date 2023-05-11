#ifndef MODEL_HPP
#define MODEL_HPP
#pragma once


namespace blackjack::model {


enum class action_e {hit, stick};

struct card {

    enum class color_e {red, black};

public:
    card(int number, color_e color)
        : number_(number), color_(color)
    { }

    operator int() {
        return color_ == color_e::black ? number_ : -number_;
    }

public:
    int number_;
    color_e color_;
};


} // namespace blackjack::model



#endif // MODEL_HPP
