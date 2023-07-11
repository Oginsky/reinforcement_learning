#ifndef AGENT_HPP
#define AGENT_HPP
#include <algorithm>
#pragma once

#include <rl/iagent.hpp>
#include <rl/utility.hpp>

#include "env.hpp"


namespace cliff {

using position = model::position;
using agent_traits = rl::TableAgentTraits<env_traits, position>;

struct Agent : public rl::IEnvAgent<Agent, agent_traits>  {

    std::map<state_t, double> value_func_;
    container_t value_action_;
    container_t policy_;

    int board_width_;
    int board_heigth_;

    Agent(int board_width, int board_heigth)
        : IEnvAgent()
        , board_width_(board_width)
        , board_heigth_(board_heigth)
    {
        reinit_impl();
    }

    void reinit_impl() {
        value_action_.clear();
        value_func_.clear();
        policy_.clear();

        for(int i = 0; i < board_width_; ++i) {
            for(int j = 0; j < board_heigth_; ++j) {
                state_t state { i, j };

                value_action_[state][action_t::up] = 0.0;
                value_action_[state][action_t::down] = 0.0;
                value_action_[state][action_t::left] = 0.0;
                value_action_[state][action_t::right] = 0.0;

                policy_[state][action_t::up] = 0.0;
                policy_[state][action_t::down] = 0.0;
                policy_[state][action_t::left] = 0.0;
                policy_[state][action_t::right] = 0.0;
            }
        }
    }

    double& value_func_impl(state_t state) {
        return value_func_[state];
    }

    double& value_action_impl(state_t state, action_t action) {
        return value_action_[state][action];
    }

    action_t best_action_impl(state_t state) {
//        return std::max_element(value_action_[state].begin(), value_action_[state].end())->first;

        double max = value_action_[state].begin()->second;
        action_t best_action = value_action_[state].begin()->first;

        for(auto it = ++value_action_[state].begin(); it != value_action_[state].end(); ++it) {
            if(it->second > max) {
                best_action = it->first;
                max = it->second;
            }
        }

        return best_action;
    }

    state_t observe_impl(observation_t observation) {
        return observation;
    }

    action_t policy_impl(state_t& state, double) {
        action_t opti = best_action_impl(state);

        if(!rand(policy_[state][opti])) {

            double prob = 1.0 / (policy_[state].size());

            for(auto it = policy_[state].begin(); it != policy_[state].end(); ++it) {
                if(it->first != opti && rand(prob)) {
                    opti = it->first;
                    break;
                }
            }
        }

        return opti;
    }

    void update_policy_impl(state_t state, double eps) {
        double factor = eps / policy_[state].size();
        action_t opti = best_action_impl(state);

        policy_[state][opti] = 1.0 - eps + factor;

        std::for_each(policy_[state].begin(), policy_[state].end(), [&factor, &opti] (auto& it) {
            if(it.first != opti)
                it.second = factor;
        });
    }

    template<typename Callable>
    void for_each_impl(Callable&& f) {
        for(auto& state_it: value_action_)
            for(auto& action_it: state_it.second)
                f(state_it.first, action_it.first);
    }

    template<typename Callable>
    void for_each_action_impl(state_t state, Callable&& f) {
        for(auto& action_it: value_action_[state])
            f(state, action_it.first);
    }

    container_t create_value_action_container_impl() {
        container_t new_container;

        for(int i = 0; i < board_width_; ++i) {
            for(int j = 0; j < board_heigth_; ++j) {
                state_t state { i, j };

                new_container[state][action_t::up] = 0.0;
                new_container[state][action_t::down] = 0.0;
                new_container[state][action_t::left] = 0.0;
                new_container[state][action_t::right] = 0.0;
            }
        }

        return new_container;
    }
};


} // namespace blackjack

#endif // AGENT_HPP
