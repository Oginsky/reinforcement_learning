#ifndef IAGENT_HPP
#define IAGENT_HPP
#pragma once

#include <rl/traits.hpp>

namespace rl {

template <typename Derived, typename Traits>
struct IAgent {

    // static_assert (rl::traits::is_agent_traits<Traits>::value, "Traits must be of type AgentTraits");

    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;

    using derived_t = Derived;

public:

    IAgent()
        : id_(next_id())
    {
        derived = static_cast<derived_t*>(this);
    }

    virtual ~IAgent() {}

    void reinit() {
        derived->reinit_impl();
    }

    double& value_func(state_t state) {
        return derived->value_func_impl(state);
    }

    template <typename Callable>
    void for_each_action(state_t state, Callable f) {
        derived->for_each_action_impl(state, std::move(f));
    }

    template <typename Callable>
    void for_each_state(Callable f) {
        derived->for_each_state_impl(std::move(f));
    }

    template<typename Callable>
    void for_each(Callable&& f) {
        derived->for_each_impl(std::move(f));
    }

    std::size_t id() const { return id_; }

protected:
    derived_t* derived;

private:
    std::size_t next_id() {
        static std::size_t ID_{0};
        return ++ID_;
    }

    std::size_t id_;

};


template <typename Derived, typename Traits>
struct IEnvAgent : public IAgent<Derived, Traits>
{
    static_assert (rl::traits::is_approximation_type_v<typename Traits::approximation_t>, "Traits must be of type AgentTraits");

    // models types
    using traits_t = typename IAgent<Derived, Traits>::traits_t;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;
    using observation_t = typename traits_t::observation_t;

    // impl
    using IAgent<Derived, Traits>::derived;

    // approximation types
    using approximation_t = typename traits_t::approximation_t;
    using container_t = typename traits::container_type<Traits>::type;
    using value_type = typename approximation_t::value_type;

public:
    IEnvAgent()
        : IAgent<Derived, Traits>()
    { }

    virtual ~IEnvAgent() {}

    value_type value_action(state_t state, action_t action) {
        return derived->value_action_impl(state, action);
    }

    action_t best_action(state_t state) {
        return derived->best_action_impl(state);
    }

    state_t observe(observation_t observation) {
        return derived->observe_impl(observation);
    }

    action_t policy(state_t state, double eps = 0.0) {
        return derived->policy_impl(state, eps);
    }

    void update_policy(state_t state, double eps) {
        derived->update_policy_impl(state, eps);
    }

    container_t create_value_action_container() {
        return derived->create_value_action_container_impl();
    }

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

    virtual ~IMDPAgent() {}

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

} // namespace rl

#endif // IAGENT_HPP
