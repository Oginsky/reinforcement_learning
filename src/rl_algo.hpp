#ifndef RL_ALGO_HPP
#define RL_ALGO_HPP
#pragma once

#include <list>

#include <detail/traits.hpp>
#include <detail/ienv.hpp>
#include <detail/iagent.hpp>


namespace rl {


struct value_iteration {

    value_iteration(double eps = 0.1e-3)
        : eps_(eps) {}

    template<typename Derived, typename Traits, typename Model>
    void operator()(IAgent<Derived, Traits>& agent, Model& model) {
        using state_t  = typename IAgent<Derived, Traits>::state_t;
        using action_t = typename IAgent<Derived, Traits>::action_t;

        double delta = eps_;
        while(delta >= eps_) {
            delta = 0;
            for(state_t state: agent.states()) {
                double v = agent.get_value_func(state);
                double max_gain = 0.0;
                action_t best_action;

                for(action_t action: agent.actions(state)) {
                    double est_gain = model.gain(state, action);
                    write_if<std::less>(max_gain, est_gain, best_action, action);
                }

                agent.set_value_func(state, max_gain);
                agent.policy()[state] = best_action;
                delta = std::max(delta, std::abs(v - agent.get_value_func(state)));
            }
        }
    }

    double eps_;
};


template<typename DerivedEnv, typename EnvTraits,
         typename CallablePolicy>
struct generate_episode {

    using observation_t = typename EnvTraits::observation_t;
    using observation_list_t = std::list<observation_t>;

public:

    generate_episode(/*const IAgent<DerivedAgent, AgentTraits>& agent,*/
                     const IEnv<DerivedEnv, EnvTraits>& env,
                     CallablePolicy policy)
        : /*agent_(agent),*/ env_(env), policy_(policy)
    { }

    observation_list_t operator()() {
        observation_list_t episode;

        env_.reset();
        observation_t obs = env_.init();
        while(true) {
            episode.push_back(obs);
            auto [state, reward, done] = obs;
            obs = env_.step(state, policy_(state));

            if(env_.is_terminal(obs)) break;
        }
        episode.push_back(obs);

        return episode;
    }

private:
    //IAgent<DerivedAgent, AgentTraits> agent_;
    IEnv<DerivedEnv, EnvTraits> env_;
    CallablePolicy policy_;
};


} // namespace rl

#endif // RL_ALGO_HPP
