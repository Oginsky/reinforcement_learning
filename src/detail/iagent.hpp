#ifndef IAGENT_HPP
#define IAGENT_HPP
#pragma once

#include <detail/traits.hpp>

template <typename Derived, typename Traits>
struct IAgent {

    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;
    using policy_t = typename traits_t::policy_t;
    using iterable_actions_t = typename traits_t::iterable_actions_t;
    using iterable_states_t = typename traits_t::iterable_states_t;

    using derived_t = Derived;

public:

    IAgent() {
        derived = static_cast<derived_t*>(this);
    }

    template <typename Callable>
    void for_each_action(state_t state, Callable f) {
        derived->for_each_action_impl(state, std::move(f));
    }

    template <typename Callable>
    void for_each_state(Callable f) {
        derived->for_each_state_impl(std::move(f));
    }

    double get_value_func(state_t state) {
        return derived->get_value_func_impl(state);
    }

    void set_value_func(state_t state, double value) {
        derived->set_value_func_impl(state, value);
    }

    state_t get_state(state_t state, action_t action) {
        derived->get_state_impl(state, action);
    }

    iterable_actions_t actions(state_t state) {
        return derived->actions_impl(state);
    }

    iterable_states_t states() {
        return derived->states_impl();
    }

    policy_t& policy() {
        return derived->policy_impl();
    }




private:
    derived_t* derived;

};



#endif // IAGENT_HPP
