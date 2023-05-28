#ifndef LINEAR_AGENT_HPP
#define LINEAR_AGENT_HPP
#pragma once

#include <vector>
#include <array>

#include <rl/iagent.hpp>
#include <rl/utility.hpp>

#include "env.hpp"


namespace blackjack {


struct LinearAgent : rl::IEnvAgent<LinearAgent, env_traits> {

    static constexpr size_t size = 36;
    using interval_t = std::pair<int, int>;
    using feature_vec_t = std::array<int, size>;
    using param_vec_t = std::array<double, size>;

    struct state_cubiod {
        int dealer;
        int player;
        action_t action;

        state_cubiod(state_t state, action_t action)
            : dealer(state.second), player(state.first), action(action)
        { }

        state_cubiod(int dealer = 0, int player = 0, action_t action = action_t::hit)
            : dealer(dealer), player(player), action(action)
        { }

        inline bool in_interval(interval_t dealer_interval, interval_t player_interval, action_t action) {
            return in_interval(dealer, dealer_interval) &&
                   in_interval(player, player_interval) &&
                   this->action == action;
        }

        private:
        inline bool in_interval(int val, interval_t interval) {
            return interval.first <= val && val <= interval.second;
        }
    };

    struct linear_approximation {
        static std::vector<interval_t> dealer_intervals;
        static std::vector<interval_t> player_intervals;

        static feature_vec_t phi(state_t state, action_t action) {
            feature_vec_t phi_res;
            state_cubiod cuboid(state, action);

            size_t interval_idx = 0;
            for(size_t di = 0, ds = dealer_intervals.size(); di < ds; ++di) {
                for(size_t pi = 0, ps = player_intervals.size(); pi < ps; ++pi) {
                    phi_res[interval_idx]   = cuboid.in_interval(dealer_intervals[di], player_intervals[pi], action_t::hit);
                    phi_res[interval_idx*2] = cuboid.in_interval(dealer_intervals[di], player_intervals[pi], action_t::stick);
                }
            }

            return phi_res;
        }
    };

public:


    LinearAgent(double eps = 0.0)
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

std::vector<LinearAgent::interval_t>
LinearAgent::linear_approximation::dealer_intervals = {{1, 4}, {4, 7}, {7, 10}};

std::vector<LinearAgent::interval_t>
LinearAgent::linear_approximation::player_intervals = {{1, 6}, {4, 9}, {7, 12}, {10, 15}, {13,18}, {16, 21}};

} // namespace blackjack

#endif // LINEAR_AGENT_HPP
