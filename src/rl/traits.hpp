#ifndef TRAITS_HPP
#define TRAITS_HPP
#pragma once

#include <tuple>
#include <map>

// fwd
namespace rl {

template <typename Observation, typename Action, typename Reward>
struct EnvTraits;

template <typename EnvTraits, typename State>
struct AgentTraits;


} // namespace rl


namespace rl::impl {

template <typename T>
struct is_env_traits : std::false_type {};

template <typename Observation, typename Action, typename Reward>
struct is_env_traits<EnvTraits<Observation, Action, Reward>> : std::true_type {};


} // namespace rl::impl


namespace rl {


template <typename Observation,
          typename Action,
          typename Reward = double>
struct EnvTraits {
    using observation_t = Observation;
    using action_t = Action;
    using reward_t = Reward;
};


template <typename EnvTraits,
          typename State>
struct AgentTraits {

    static_assert (rl::impl::is_env_traits<EnvTraits>::value, "Traits must be of type EnvTraits");

    using env_traits = EnvTraits;
    using observation_t = typename EnvTraits::observation_t;
    using action_t = typename EnvTraits::action_t;
    using reward_t = typename EnvTraits::reward_t;
    using state_t = State;
};


template <typename Action,
          typename State,
          typename Reward = double,
          typename Observation = State>
struct Traits {
    using action_t = Action;
    using state_t = State;
    using reward_t = Reward;
    using observation_t = Observation;
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


} // namespace rl

#endif // TRAITS_HPP
