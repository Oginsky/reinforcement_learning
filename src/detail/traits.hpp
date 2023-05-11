#ifndef TRAITS_HPP
#define TRAITS_HPP
#pragma once


// fwd
namespace std {

}
template <typename T>
struct null_iterable;

template <typename Action,
          typename State,
          template<typename> class IterableA,
          template<typename> class IterableS = IterableA,
          typename Policy = std::map<State, Action>>
struct Traits {
    using action_t = Action;
    using state_t = State;
    using policy_t = Policy;
    using iterable_actions_t = IterableA<Action>;
    using iterable_states_t = IterableS<State>;
};



template <typename Action,
          typename State,
          typename Policy = std::map<State, Action>>
struct BaseTraits
{
    using action_t = Action;
    using state_t = State;
    using policy_t = Policy;
};


template <typename BaseTraits,
          template<typename> class IterableA,
          template<typename> class IterableS>
struct AgentTraits
{
    using action_t = typename BaseTraits::action_t;
    using state_t = typename BaseTraits::state_t;
    using policy_t = typename BaseTraits::policy_t;
    using iterable_actions_t = IterableA<action_t>;
    using iterable_states_t = IterableS<state_t>;
};

template <typename BaseTraits,
          typename Reward = double>
struct EnvTraits
{
    using action_t = typename BaseTraits::action_t;
    using state_t = typename BaseTraits::state_t;
    using policy_t = typename BaseTraits::policy_t;
    using reward_t = Reward;
    using observation_t = std::tuple<state_t, reward_t, bool>;
};


#endif // TRAITS_HPP
