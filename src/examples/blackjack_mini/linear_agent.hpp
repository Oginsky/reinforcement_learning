#ifndef LINEAR_AGENT_HPP
#define LINEAR_AGENT_HPP
#pragma once

#include <vector>
#include <array>

#include <rl/iagent.hpp>
#include <rl/utility.hpp>
#include <rl/linear_vector.hpp>

#include "env.hpp"


namespace blackjack {

using interval_t = std::pair<int, int>;
using action_t = env_traits::action_t;
using state_t = env_traits::state_t;

static std::vector<interval_t> dealer_intervals = {{1, 4}, {4, 7}, {7, 10}};
static std::vector<interval_t> player_intervals = {{1, 6}, {4, 9}, {7, 12}, {10, 15}, {13,18}, {16, 21}};

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

    static constexpr size_t APPROXIMATE_SIZE = 36;

    using param_vec_t = std::vector<double>;
    using feature_vec_t = std::vector<bool>;
    using approx_state_t = rl::linear_vector<APPROXIMATE_SIZE, param_vec_t>;

public:
    linear_approximation()
        : data_(APPROXIMATE_SIZE, 0.0) { }

public:

    static feature_vec_t phi(state_t state, action_t action) {
        feature_vec_t phi_res(APPROXIMATE_SIZE);
        state_cubiod cuboid(state, action);

        size_t interval_idx = 0;
        for(size_t di = 0, ds = dealer_intervals.size(); di < ds; ++di) {
            for(size_t pi = 0, ps = player_intervals.size(); pi < ps; ++pi) {
                phi_res[interval_idx]   = cuboid.in_interval(dealer_intervals[di], player_intervals[pi], action_t::hit);
                phi_res[interval_idx+1] = cuboid.in_interval(dealer_intervals[di], player_intervals[pi], action_t::stick);
                interval_idx += 2;
            }
        }

        return phi_res;
    }

public:

    approx_state_t operator()(state_t state, action_t action) {
        return approx_state_t(phi(state, action), data_);
    }

private:
    param_vec_t data_;

};

struct LinearAgent : rl::IApproxAgent<LinearAgent, env_traits, linear_approximation> {

public:
    LinearAgent(double eps = 0.0)
        : IApproxAgent(eps)
    {
        reinit_impl();
    }

    void reinit_impl() {

    }

    approx_state_t value_action_impl(state_t state, action_t action) {
        return value_action(state, action);
    }

    action_t get_best_action_impl(state_t state) {
        double q_hit   = value_action(state, action_t::hit),
               q_stick = value_action(state, action_t::stick);
        return (q_hit > q_stick) ? action_t::hit : action_t::stick;
    }

    state_t get_state_impl(observation_t observation) {
        return observation;
    }

    action_t policy_impl(observation_t observation) {
        state_t state = observation;
        action_t subopti = get_best_action_impl(state);
        action_t other = (subopti == action_t::hit) ? action_t::stick : action_t::hit;

        return rand(eps_) ? other : subopti;
    }

    template<typename Callable>
    void for_each_impl(Callable&& f) {
        for(auto& player_int: player_intervals) {
            for(auto& dealer_int: dealer_intervals) {
                state_t state{player_int.first, dealer_int.second};
                f(state, action_t::hit);
                f(state, action_t::stick);
            }
        }

    }

public:
    linear_approximation value_action;

};

} // namespace blackjack

#endif // LINEAR_AGENT_HPP
