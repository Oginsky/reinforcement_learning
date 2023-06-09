#ifndef IENV_HPP
#define IENV_HPP
#pragma once

#include <tuple>

#include <rl/traits.hpp>


namespace rl {

//fwd
template <typename Derived, typename Traits>
struct IAgent;


template <typename Derived, typename Traits>
struct IEnv {
    using derived_t = Derived;

    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using reward_t = typename traits_t::reward_t;
    using observation_t = typename traits_t::observation_t;
    using step_tuple_t = std::tuple<observation_t, reward_t, bool>;

public:
    // fwd
    struct step;
    using step_t = step;

    struct step {
        observation_t obs;
        reward_t reward;
        bool done;

        step() { }
        step(observation_t, reward_t reward, bool done)
            : obs{obs}, reward(reward), done(done) { }

        step(step_tuple_t tuple) {
            obs = std::get<0>(tuple);
            reward = std::get<1>(tuple);
            done = std::get<2>(tuple);
        }

        operator step_tuple_t() const {
            return std::make_tuple(obs, reward, done);
        }
    };


public:
    IEnv(){
        derived = static_cast<derived_t*>(this);
    }

    virtual ~IEnv() {}

    template <typename DerivedAgent, typename AgentTraits>
    step_t init(const IAgent<DerivedAgent, AgentTraits>& agent) {
        return derived->init_impl(agent.id());
    }

    template <typename DerivedAgent, typename AgentTraits>
    step_t step(const IAgent<DerivedAgent, AgentTraits>& agent, const action_t& action) {
        return derived->step_impl(agent.id(), action);
    }

    void reset() {
        return derived->reset_impl();
    }

private:
    derived_t* derived;
};


} // namespace rl

#endif // IENV_HPP
