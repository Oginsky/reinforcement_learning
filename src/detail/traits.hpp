#ifndef TRAITS_HPP
#define TRAITS_HPP
#pragma once


// fwd
namespace std {

}

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

#endif // TRAITS_HPP
