#ifndef TRAITS_HPP
#define TRAITS_HPP
#pragma once

#include <tuple>
#include <map>



namespace rl::traits {

// fwd
template <typename Observation, typename Action, typename Reward>
struct EnvTraits;

template <typename EnvTraits, typename State, typename Approximation>
struct AgentTraits;


// used types
struct null_approximation_t {
    using value_type = double&;
    template <class K, class V> using container = std::map<K, V>;
};


// utility types
template <typename T>
struct is_env_traits : std::false_type {};

template <typename Observation, typename Action, typename Reward>
struct is_env_traits<EnvTraits<Observation, Action, Reward>> : std::true_type {};

template <typename T>
struct is_agent_traits : std::false_type {};

template <typename EnvTraits, typename State, typename Approximation>
struct is_agent_traits<AgentTraits<EnvTraits, State, Approximation>> : std::true_type {};

template <typename T, typename = void>
struct is_approximation_type : std::false_type {};

template <typename T>
struct is_approximation_type<T, std::void_t<typename T::value_type>> : std::true_type {};

template <typename T>
constexpr bool is_approximation_type_v = is_approximation_type<T>::value;

template <typename Traits>
struct is_table_based_agent {
    static constexpr bool value = std::is_same_v<typename Traits::approximation_t, null_approximation_t>;
};

template <typename Traits>
struct container_type {
    static_assert (rl::traits::is_agent_traits<Traits>::value, "Traits must be of type AgentTraits");

    using type = typename Traits::approximation_t;
};

template <typename EnvTraits, typename State>
struct container_type<AgentTraits<EnvTraits, State, null_approximation_t>> {
    using type = null_approximation_t::container<
                                            State, null_approximation_t::container<
                                            typename EnvTraits::action_t, double>
                                                >;
};


// Type Traits structure
template <typename Observation,
          typename Action,
          typename Reward = double>
struct EnvTraits {
    using observation_t = Observation;
    using action_t = Action;
    using reward_t = Reward;
};


template <typename EnvTraits,
          typename State,
          typename Approximation = null_approximation_t>
struct AgentTraits {

    static_assert (rl::traits::is_env_traits<EnvTraits>::value, "Traits must be of type EnvTraits");

    using env_traits = EnvTraits;
    using observation_t = typename EnvTraits::observation_t;
    using action_t = typename EnvTraits::action_t;
    using reward_t = typename EnvTraits::reward_t;
    using state_t = State;
    using approximation_t = Approximation;
};


template <typename Action,
          typename State,
          template<typename> class IterableA,
          template<typename> class IterableS,
          typename Policy = std::map<State, Action>>
struct MDPTraits
{
    using action_t = Action;
    using state_t = State;
    using policy_t = Policy;
    using iterable_actions_t = IterableA<action_t>;
    using iterable_states_t = IterableS<state_t>;
};



} // namespace rl::traits


namespace rl {

using rl::traits::EnvTraits;
using rl::traits::AgentTraits;
using rl::traits::MDPTraits;

template <typename EnvTraits,
          typename State>
using TableAgentTraits = AgentTraits<EnvTraits, State, traits::null_approximation_t>;


} // namespace rl

#endif // TRAITS_HPP
