#ifndef TRAITS_HPP
#define TRAITS_HPP
#pragma once

#include <tuple>


template <typename Action,
          typename State,
          typename Reward = double,
          typename Observation = std::tuple<State, Reward, bool>>
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

#endif // TRAITS_HPP
