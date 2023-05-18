#ifndef IENV_HPP
#define IENV_HPP
#pragma once

#include <tuple>

#include <detail/traits.hpp>


template <typename Derived, typename Traits>
struct IEnv {
    using derived_t = Derived;

    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using state_t  = typename traits_t::state_t;
    using reward_t = typename traits_t::reward_t;
    using observation_t = typename traits_t::observation_t;
    using step_result_t = std::tuple<observation_t, reward_t, bool>;

public:
    IEnv(){
        derived = static_cast<derived_t*>(this);
    }

    observation_t init() {
        return derived->init_impl();
    }

    observation_t step(const state_t& state, const action_t& action) {
        return derived->step_impl(state, action);
    }

    bool is_terminal(const observation_t& observation) {
        return derived->is_terminal_impl(observation);
    }

    void reset() {
        return derived->reset_impl();
    }

private:
    derived_t* derived;
};


#endif // IENV_HPP
