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
#include <utility.hpp>


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
                double v = agent.value_func(state);
                double max_gain = 0.0;
                action_t best_action;

                for(action_t action: agent.actions(state)) {
                    double est_gain = model.gain(state, action);
                    write_if<std::less>(max_gain, est_gain, best_action, action);
                }

                agent.value_func(state) = max_gain;
                agent.policy()[state] = best_action;
                delta = std::max(delta, std::abs(v - agent.value_func(state)));
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
    using state_t = typename Traits::state_t;
    using observation_t = typename Traits::observation_t;
    using step_t = typename IEnv<DerivedEnv, Traits>::step_t;

    struct result {
        step_t step;
        action_t action;
    };

    using episode_t = std::vector<result>;

public:

    generate_episode(const IEnvAgent<DerivedAgent, Traits>& agent,
                     const IEnv<DerivedEnv, Traits>& env,
                     size_t count)
        : agent_(agent), env_(env), current_(0), count_(count)
    { }

    episode_t operator()() {
        episode_t episode;

        env_.reset();
        step_t step = env_.init();
        action_t action;
        while(!step.done) {
            action = agent_.policy(step.obs);
            episode.push_back({step, action});

            step = env_.step(step.obs, action);
            episode.back().step.reward = step.reward;
        }

        return episode;
    }

    bool operator>>(episode_t& episode) {
        if(current_ >= count_) return false;

        episode_t{}.swap(episode); // clear ptr
        episode = std::move((*this)());
        current_++;

        return true;
    }

private:
    IEnvAgent<DerivedAgent, Traits> agent_;
    IEnv<DerivedEnv, Traits> env_;
    size_t current_{0};
    size_t count_{0};
};


template<typename Traits,
         typename DerivedAgent,
         typename DerivedEnv>
struct first_visit_mc_prediction {

    using state_t = typename Traits::state_t;

public:
    first_visit_mc_prediction(IEnvAgent<DerivedAgent, Traits>& agent,
                              IEnv<DerivedEnv, Traits>& env,
                              size_t n_episodes,
                              double discont = 1.0)
        : agent_(agent), env_(env), n_episodes_(n_episodes), discont_(discont)
    { }

    void operator()() {
        std::unordered_map<state_t, int> N;
        generate_episode episodes_gen(agent_, env_, n_episodes_);
        typename decltype(episodes_gen)::episode_t episode;

        while(episodes_gen >> episode) {
            double gain = 0.0;
            int obs_idx = episode.size() - 1;
            for(auto obs_it = episode.rbegin(); obs_it != episode.rend(); ++obs_it, --obs_idx) {
                auto [state, reward, done] = (*obs_it).step;

                gain = discont_*gain + reward;
                auto state_it = std::find_if(episode.begin(), episode.begin()+obs_idx,
                        [state](const auto& step) {return step.step.obs == state;});

                if(state_it == episode.end()) {
                    N[state]++;
                    double value = (gain - agent_.value_func(state)) / N[state];
                    agent_.value_func(state) = value;
                }
            }
        }
    }

private:
    IEnvAgent<DerivedAgent, Traits> agent_;
    IEnv<DerivedEnv, Traits> env_;
    int n_episodes_;
    double discont_;
};


template <typename Episode, typename State, typename Action>
auto first_of(const Episode& episode, const State& state, const Action& action) {
    const auto compare = [state, action](const typename Episode::value_type& step) {
        State step_state = step.step.obs;
        Action step_action = step.action;

        return state == step_state && action == step_action;
    };

    return std::find_if(begin(episode), end(episode), compare);
}

template<typename Traits,
        typename DerivedAgent,
        typename DerivedEnv>
void first_visit_mc_control(IEnvAgent<DerivedAgent, Traits>& agent,
                            IEnv<DerivedEnv, Traits>& env,
                            double eps,
                            int n_episodes,
                            double discont = 1.0)
{
    using state_t = typename Traits::state_t;
    using action_t = typename Traits::action_t;
    using observation_t = typename Traits::observation_t;
    using step_t = typename IEnv<DerivedEnv, Traits>::step_t;

    std::unordered_map<state_t, std::unordered_map<action_t, int>> N;
    generate_episode episodes_gen{agent, env, n_episodes};
    typename decltype(episodes_gen)::episode_t episode;

    while(episodes_gen >> episode) {
        double gain{0.0};
        for(auto step_it = episode.rbegin(); step_it != episode.rend(); ++step_it) {
            auto [obs, reward, done] = (*step_it).step;
            state_t state = obs;
            action_t action = (*step_it).action;
            gain = discont*gain + reward;

            auto it = first_of(episode, state, action);
            if(std::make_reverse_iterator(++it) == step_it) {
                int n = ++N[state][action];
                double q = agent.value_action(state, action);
                agent.value_action(state, action) = q + (1.0 / n ) * (gain - q);
                agent.learn(state);
            }
        }
    }
}

} // namespace rl

#endif // RL_ALGO_HPP
