#ifndef ENV_HPP
#define ENV_HPP
#pragma once

#include <map>
#include <random>

#include <detail/traits.hpp>
#include <detail/iagent.hpp>
#include <detail/ienv.hpp>
#include <detail/iterable_sequence.hpp>

#include <blackjack/model.hpp>

#include <utility.hpp>


namespace std {

template<>
struct hash<std::pair<int, int>> {
    std::size_t operator()(const std::pair<int, int>& key) const {
        return std::hash<int>{}(key.first) ^ (std::hash<int>{}(key.second) << 1);
    }
};

}


namespace blackjack {

using env_traits = Traits<model::action_e, std::pair<int, int>>;


struct Env : IEnv<Env, env_traits> {

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
                return (player_points <= 21) ? std::make_tuple(new_state, 0.0, false)
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


struct Agent : public IEnvAgent<Agent, env_traits>  {


    Agent(double eps = 0.0)
        : IEnvAgent(eps)
    {

        for(int i = 0; i < 21; ++i) {
            for(int j = 1; j < 10; ++j) {
                state_t state{i, j};
                value_action[state][action_t::hit] = rand()%100;
                value_action[state][action_t::stick] = rand()%100;
            }
        }

    }

    double& value_func_impl(state_t state) {
        return value_func[state];
    }

    double& value_action_impl(state_t state, action_t action) {
        return value_action[state][action];
    }

    action_t get_best_action_impl(state_t state) {
        return (value_action[state][action_t::hit] > value_action[state][action_t::stick])
                ? action_t::hit : action_t::stick;
    }

    state_t get_state_impl(observation_t observation) {
        return observation;
    }

    action_t policy_impl(observation_t observation) {
        state_t state = observation;
        action_t subopti = get_best_action_impl(state);
        action_t other = (subopti == action_t::hit) ? action_t::stick : action_t::hit;

        return rand(policy_[state][other]) ? other : subopti;
    }

    void learn_impl(state_t state) {
        action_t subopti = get_best_action_impl(state);
        action_t other = (subopti == action_t::hit) ? action_t::stick : action_t::hit;

        policy_[state][subopti] = 1.0 - eps_ + eps_ / 2.0;
        policy_[state][other] = eps_ / 2.0;
    }

public:
    std::map<state_t, double> value_func;
    std::map<state_t, std::map<action_t, double>> value_action;
    std::map<state_t, std::map<action_t, double>> policy_;
};

} // namespace blackjack

#endif // ENV_HPP
