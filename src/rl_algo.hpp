#ifndef ALGO_HPP
#define ALGO_HPP
#pragma once

#include <detail/traits.hpp>
#include <detail/iagent.hpp>


namespace rl {


struct value_iteration {

    value_iteration(double eps = 0.1e-3)
        : eps_(eps) {}

    template<typename Derived, typename Traits, typename Model>
    void operator()(IAgent<Derived, Traits>& agent, Model& model) {
        using state_t  = typename IAgent<Derived, Traits>::state_t;
        using action_t = typename IAgent<Derived, Traits>::action_t;

        double delta = eps_;
        while(delta >= eps_) {
            delta = 0;
            for(state_t state: agent.states()) {
                double v = agent.get_value_func(state);
                double max_gain = 0.0;
                action_t best_action;

                for(action_t action: agent.actions(state)) {
                    double est_gain = model.gain(state, action);
                    write_if<std::less>(max_gain, est_gain, best_action, action);
                }

                agent.set_value_func(state, max_gain);
                agent.policy()[state] = best_action;
                delta = std::max(delta, std::abs(v - agent.get_value_func(state)));
            }
        }
    }

    double eps_;
};


} // namespace rl

#endif // ALGO_HPP
