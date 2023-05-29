#ifndef AGENT_HPP
#define AGENT_HPP
#pragma once

#include <rl/iagent.hpp>
#include <rl/utility.hpp>

#include "env.hpp"


namespace blackjack {


struct Agent : public rl::IEnvAgent<Agent, env_traits>  {


    Agent(double eps = 0.0)
        : IEnvAgent(eps)
    {
        reinit_impl();
    }

    void reinit_impl() {
        value_action.clear();
        value_func.clear();
        policy_.clear();

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

    void update_policy_impl(state_t state, double eps) {
        action_t subopti = get_best_action_impl(state);
        action_t other = (subopti == action_t::hit) ? action_t::stick : action_t::hit;

        policy_[state][subopti] = 1.0 - eps + eps / 2.0;
        policy_[state][other] = eps / 2.0;
    }

    template<typename Callable>
    void for_each_impl(Callable&& f) {
        for(auto& state_it: value_action)
            for(auto& action_it: state_it.second)
                f(state_it.first, action_it.first);
    }

public:
    std::map<state_t, double> value_func;
    std::map<state_t, std::map<action_t, double>> value_action;
    std::map<state_t, std::map<action_t, double>> policy_;
};


} // namespace blackjack

#endif // AGENT_HPP
