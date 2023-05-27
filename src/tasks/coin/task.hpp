#ifndef TASK_HPP
#define TASK_HPP
#pragma once

#include <vector>
#include <map>
#include <functional>
#include <random>
#include <algorithm>

#include <detail/iagent.hpp>
#include <detail/iterable_sequence.hpp>

#include <utility.hpp>

using std::vector;
using std::map;


struct Task {

public:
    using agent_traits = MDPTraits<int, int, iterable_sequence, iterable_sequence>;

    const double reward{0.0};
    const double eps{1.e-5};
    const double discont{1.0};

public:
    struct Agent : IMDPAgent<Agent, agent_traits>
    {

        double& value_func_impl(state_t state) {
            return v[state];
        }

        state_t get_state_impl(state_t state, action_t action) {
            return state_t{state + action};
        }

        iterable_actions_t actions_impl(state_t state) {
            return iterable_actions_t(action_t{1}, action_t{std::min(state, 100-state)});
        }

        iterable_states_t states_impl() {
            return iterable_states_t(state_t{1}, state_t{99});
        }

        policy_t& policy_impl() {
            return policy_;
        }

        vector<double> v;
        policy_t policy_;

    };

public:
    const double ph_;

    Task(double ph) : ph_(ph)
    {
        init();
    }

    void init() {
        constexpr size_t n = 100;
        std::mt19937_64 engine{std::random_device{}()};
        std::uniform_int_distribution<int64_t> distr{0, 1000};

        agent.v.reserve(n);
        std::generate_n(std::back_inserter(agent.v), n, std::bind(distr, engine));
        agent.v[0] = 0;
    }

    double gain(Agent::state_t state, Agent::action_t action) {
        Agent::state_t s1 = state + action,
                       s2 = state - action;

        return is_terminal_state(s1)
                ? ph_ + (1.0 - ph_)*(reward + discont*agent.value_func(s2))

                : ph_*(reward + discont*agent.value_func(s1))
                  +(1.0 - ph_)*(reward + discont*agent.value_func(s2));
    }

    bool is_terminal_state(Agent::state_t state) {
        return state == 100;
    }

public:
    Agent agent;

};

#endif // TASK_HPP
