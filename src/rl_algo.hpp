#ifndef RL_ALGO_HPP
#define RL_ALGO_HPP
#pragma once

#include <unordered_map>
#include <algorithm>
#include <list>
#include <vector>

#include <detail/traits.hpp>
#include <detail/ienv.hpp>
#include <detail/iagent.hpp>


namespace rl {


struct value_iteration {

    value_iteration(double eps = 0.1e-3)
        : eps_(eps) {}

    template<typename Derived, typename Traits, typename Model>
    void operator()(IMDPAgent<Derived, Traits>& agent, Model& model) {
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


template<typename Traits,
         typename DerivedAgent,
         typename DerivedEnv>
struct generate_episode {

    using action_t = typename Traits::action_t;
    using observation_t = typename Traits::observation_t;

    struct result {
        observation_t observation;
        action_t action;
    };

    using observation_list_t = std::vector<result>;

public:

    generate_episode(const IEnvAgent<DerivedAgent, Traits>& agent,
                     const IEnv<DerivedEnv, Traits>& env)
        : agent_(agent), env_(env)
    { }

    observation_list_t operator()() {
        observation_list_t episode;

        env_.reset();
        observation_t obs = env_.init();
        action_t action;
        while(true) {
            auto [state, reward, done] = obs;
            action = agent_.policy(obs);
            episode.push_back({obs, action});
            obs = env_.step(state, action);

            if(env_.is_terminal(obs)) break;
        }
        episode.push_back({obs, action});

        return episode;
    }

private:
    IEnvAgent<DerivedAgent, Traits> agent_;
    IEnv<DerivedEnv, Traits> env_;
};


template<typename Traits,
         typename DerivedAgent,
         typename DerivedEnv>
struct first_visit_mc_prediction {

    using state_t = typename Traits::state_t;

public:
    first_visit_mc_prediction(const IAgent<DerivedAgent, Traits>& agent,
                              const IEnv<DerivedEnv, Traits>& env,
                              int n_episodes,
                              double discont = 1.0)
        : agent_(agent), env_(env), n_episodes_(n_episodes), discont_(discont)
    {
        std::unordered_map<state_t, int> N;
        generate_episode episodes_gen{env, agent.policy};

        for(int n = 0; n < n_episodes; ++n) {
            auto episode = episodes_gen();
            double gain = 0.0;
            int obs_idx = episode.size() - 1;
            for(auto obs_it = episode.rbegin(); obs_it != episode.rend(); ++obs_it, --obs_idx) {
                auto [state, reward, done] = *obs_it.observation;

                gain = discont_*gain + reward;
                auto state_it = std::find_if(episode.begin(), episode.begin()+obs_idx,
                        [state](const auto& obs) {return obs.first == state;});

                if(state_it == episode.end()) {
                    N[state]++;
                    double value = (gain - agent.get_value_func(state)) / N[state];
                    agent.set_value_func(state, value);
                }
            }
        }
    }

private:
    IAgent<DerivedAgent, Traits> agent_;
    IEnv<DerivedEnv, Traits> env_;
    int n_episodes_;
    double discont_;
};


template<typename Traits,
        typename DerivedAgent,
        typename DerivedEnv>
struct first_visit_mc_control {

    using state_t = typename Traits::state_t;
    using action_t = typename Traits::action_t;
    using observation_t = typename Traits::observation_t;

public:
    first_visit_mc_control(const IEnvAgent<DerivedAgent, Traits>& agent,
                           const IEnv<DerivedEnv, Traits>& env,
                           double eps,
                           int n_episodes,
                           double discont = 1.0)
        : agent_(agent), env_(env), eps_(eps), n_episodes_(n_episodes), discont_(discont)
    {

    }

    void operator()() {
        std::unordered_map<state_t, std::unordered_map<action_t, int>> N;
        generate_episode episodes_gen{agent_, env_};
        for(int n = 0; n < n_episodes_; ++n) {
            auto episode = episodes_gen();
            double gain{0.0};
            for(auto step_it = episode.rbegin(); step_it != episode.rend(); ++step_it) {
                auto [state, reward, done] = (*step_it).observation;
                action_t action = (*step_it).action;
                gain = discont_*gain + reward;

                auto it = first_of(episode, state, action);
                if(std::make_reverse_iterator(++it) == step_it) {
                    int n = ++N[state][action];
                    double q = agent_.get_value_action(state, action);
                    agent_.set_value_action(state, action, q + (1.0 / n ) * (gain - q));
                    agent_.learn(state);
                }
            }
        }
    }


    template <typename Episode>
    auto first_of(const Episode& episode, const state_t& state, const action_t& action) {
        const auto compare = [state, action](const typename Episode::value_type& step) {
            state_t step_state = std::get<0>(step.observation);
            action_t step_action = step.action;

            return state == step_state && action == step_action;
        };

        return std::find_if(begin(episode), end(episode), compare);
    }


private:
    IEnvAgent<DerivedAgent, Traits> agent_;
    IEnv<DerivedEnv, Traits> env_;
    double eps_;
    int n_episodes_;
    double discont_;
};


} // namespace rl

#endif // RL_ALGO_HPP
