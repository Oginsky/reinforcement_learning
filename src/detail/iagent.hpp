#ifndef IAGENT_HPP
#define IAGENT_HPP
#pragma once

#include <detail/traits.hpp>

template <typename Derived, typename Traits>
struct IAgent {

    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;

    using derived_t = Derived;

public:

    IAgent() {
        derived = static_cast<derived_t*>(this);
    }

    double& value_func(state_t state) {
        return derived->value_func_impl(state);
    }

protected:
    derived_t* derived;

};


template <typename Derived, typename Traits>
struct IEnvAgent : public IAgent<Derived, Traits>
{
    using traits_t = typename IAgent<Derived, Traits>::traits_t;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;
    using observation_t = typename traits_t::observation_t;
    using IAgent<Derived, Traits>::derived;

public:
    IEnvAgent(double eps = 0.0)
        : IAgent<Derived, Traits>(),
          eps_(eps)
    { }

    double& value_action(state_t state, action_t action) {
        return derived->value_action_impl(state, action);
    }

    action_t get_best_action(state_t state) {
        return derived->get_best_action_impl(state);
    }

    state_t get_state(observation_t observation) {
        return derived->get_state_impl(observation);
    }

    action_t policy(observation_t observation) {
        return derived->policy_impl(observation);
    }

    void learn(state_t state) {
        derived->learn_impl(state);
    }

public:
    double eps_;
};

template <typename Derived, typename Traits>
struct IMDPAgent : public IAgent<Derived, Traits>
{

    using traits_t = typename IAgent<Derived, Traits>::traits_t;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;
    using policy_t = typename traits_t::policy_t;
    using iterable_actions_t = typename traits_t::iterable_actions_t;
    using iterable_states_t = typename traits_t::iterable_states_t;

    using IAgent<Derived, Traits>::derived;

public:
    IMDPAgent() : IAgent<Derived, Traits>() { }


    template <typename Callable>
    void for_each_action(state_t state, Callable f) {
        derived->for_each_action_impl(state, std::move(f));
    }

    template <typename Callable>
    void for_each_state(Callable f) {
        derived->for_each_state_impl(std::move(f));
    }

    iterable_actions_t actions(state_t state) {
        return derived->actions_impl(state);
    }

    iterable_states_t states() {
        return derived->states_impl();
    }

    state_t get_state(state_t state, action_t action) {
        derived->get_state_impl(state, action);
    }

    policy_t& policy() {
        return derived->policy_impl();
    }

};

#endif // IAGENT_HPP
