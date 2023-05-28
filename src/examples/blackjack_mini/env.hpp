#ifndef ENV_HPP
#define ENV_HPP
#pragma once

#include <map>
#include <random>

#include <rl/traits.hpp>
#include <rl/ienv.hpp>

#include "model.hpp"


namespace std {

template<>
struct hash<std::pair<int, int>> {
    std::size_t operator()(const std::pair<int, int>& key) const {
        return std::hash<int>{}(key.first) ^ (std::hash<int>{}(key.second) << 1);
    }
};

}


namespace blackjack {

using env_traits = rl::Traits<model::action_e, std::pair<int, int>>;


struct Env : rl::IEnv<Env, env_traits> {

    static constexpr double discont{1.0};

public:
    Env()
        : engine{std::random_device{}()},
          distr(1, 10)
    {

    }

    step_t init_impl() {
        model::card player_card = take_card(model::card::color_e::black),
                    diler_card = take_card(model::card::color_e::black);
        diler_points += diler_card;

        state_t state = std::make_pair(static_cast<int>(player_card), static_cast<int>(diler_card));
        return std::make_tuple(state, 0.0, false);
    }

    step_t step_impl(const state_t& state, const action_t& action) {
        int player_points = state.first;
        switch (action) {
            case action_t::hit:
            {
                player_points += take_card();
                state_t new_state = std::make_pair(player_points, state.second);
                return (player_points <= 21 && player_points > 0)
                                             ? std::make_tuple(new_state, 0.0, false)
                                             : std::make_tuple(new_state, -1.0, true);
            }

            case action_t::stick:
            {
                while(diler_points < 17 && diler_points > 0)
                    diler_points += take_card();

                state_t new_state = std::make_pair(player_points, diler_points);
                if(diler_points == player_points) return std::make_tuple(new_state, 0.0, true);

                return (player_points > diler_points || diler_points < 0 || diler_points > 21)
                        ? std::make_tuple(new_state, 1.0, true)
                        : std::make_tuple(new_state, -1.0, true);
            }
        }
        return {};
    }

    void reset_impl() {
        diler_points = 0;
    }

    model::card take_card() {
        int number = distr(engine);
        model::card::color_e color = (distr(engine) % 3 == 0)
                                     ? model::card::color_e::red
                                     : model::card::color_e::black;
        return model::card{number, color};
    }

    model::card take_card(model::card::color_e color) {
        int number = distr(engine);
        return model::card{number, color};
    }

public:
    std::mt19937_64 engine{std::random_device{}()};
    std::uniform_int_distribution<int64_t> distr{0, 1000};
    int diler_points = 0;
};

} // namespace blackjack

#endif // ENV_HPP