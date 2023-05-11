#ifndef ENV_HPP
#define ENV_HPP
#pragma once

#include <map>
#include <random>

#include <detail/traits.hpp>
#include <detail/ienv.hpp>
#include <detail/iterable_sequence.hpp>

#include <blackjack/model.hpp>


namespace blackjack {

using base_traits = BaseTraits<model::action_e, std::pair<int, int>>;
using env_traits = EnvTraits<base_traits, int>;


struct Env : IEnv<Env, env_traits> {

    Env()
        : engine{std::random_device{}()},
          distr(1, 10)
    {

    }

    observation_t init_impl() {
        model::card player_card = take_card(model::card::color_e::black),
                    diler_card = take_card(model::card::color_e::black);
        diler_points += diler_card;

        state_t state = std::make_pair(static_cast<int>(player_card), static_cast<int>(diler_card));
        return std::make_tuple(state, 0.0, false);
    }

    observation_t step_impl(const state_t& state, const action_t& action) {
        int player_points = state.first;
        switch (action) {
            case action_t::hit:
            {
                player_points += take_card();
                state_t new_state = std::make_pair(player_points, state.second);
                return (player_points <= 21) ? std::make_tuple(new_state, 0, false)
                                             : std::make_tuple(new_state, -1, true);
            }

            case action_t::stick:
            {
                while(diler_points < 17 && diler_points > 0)
                    diler_points += take_card();

                state_t new_state = std::make_pair(player_points, diler_points);
                if(diler_points == player_points) return std::make_tuple(new_state, 0, true);

                return (player_points > diler_points || diler_points < 0)
                        ? std::make_tuple(new_state, 1, true)
                        : std::make_tuple(new_state, -1, true);
            }
        }
        return {};
    }

    bool is_terminal_impl(const observation_t& observation) {
        return std::get<2>(observation);
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
