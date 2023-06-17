#ifndef RL_ALGO_HPP
#define RL_ALGO_HPP
#pragma once

#include <unordered_map>
#include <algorithm>
#include <list>
#include <vector>
#include <functional>

#include <rl/traits.hpp>
#include <rl/ienv.hpp>
#include <rl/iagent.hpp>
#include <rl/utility.hpp>


namespace rl::algorithms {


struct parameters_set {

    using lambda_t = std::function<double(int n)>;

    parameters_set() = delete;
    parameters_set(const parameters_set& other) = default;
    parameters_set(parameters_set&& other) = default;
    ~parameters_set() = default;

    template <typename Func1, typename Func2>
    parameters_set(double discont, size_t episodes, Func1&& rate, Func2&& prob)
        : discont(discont), episodes_count(episodes),
          learning_rate(std::move(rate)), exploration_prob(std::move(prob))
    { }

public:
    double discont;
    size_t episodes_count;
    lambda_t learning_rate;
    lambda_t exploration_prob;

public:
    parameters_set& set_discont(double val) { discont = val; return *this; }
    parameters_set& set_episodes(size_t val) { episodes_count = val; return *this; }

    template <typename Func>
    parameters_set& set_learning_rate(Func&& lambda) {
        learning_rate = std::move(lambda);
        return *this;
    }
    template <typename Func>
    parameters_set& set_exploration_prob(Func&& lambda) {
        exploration_prob = std::move(lambda);
        return *this;
    }

};

parameters_set create_default_params_set(double discont, size_t episodes, double rate, double prob) {
    return parameters_set(discont, episodes,
                          [rate](int){return rate;},
                          [prob](int){return prob;});
}

struct value_iteration {

    value_iteration(double eps = 0.1e-6)
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
                //agent.policy()[state] = best_action;
                delta = std::max(delta, std::abs(v - agent.value_func(state)));
            }
        }

        // reconstruction policy
        for(state_t state: agent.states()) {
            double max_gain = 0.0;
            action_t best_action;
            for(action_t action: agent.actions(state)) {
                double est_gain = model.gain(state, action);
                write_if<std::less>(max_gain, est_gain, best_action, action);
            }
            agent.policy()[state] = best_action;
        }
    }

    double eps_;
};


template<typename AgentTraits,
         typename EnvTraits,
         typename DerivedAgent,
         typename DerivedEnv>
struct generate_episode {

    using agent_traits = AgentTraits;
    using env_traits = EnvTraits;
    using action_t = typename AgentTraits::action_t;
    using state_t = typename AgentTraits::state_t;
    using observation_t = typename AgentTraits::observation_t;
    using step_t = typename IEnv<DerivedEnv, EnvTraits>::step_t;

    struct result {
        step_t step;
        action_t action;
    };

    using episode_t = std::vector<result>;

public:

    generate_episode(const IEnvAgent<DerivedAgent, agent_traits>& agent,
                     const IEnv<DerivedEnv, env_traits>& env,
                     size_t count)
        : agent_(agent), env_(env), current_(0), count_(count)
    { }

    episode_t operator()() {
        episode_t episode;

        env_.reset();
        step_t step = env_.init(agent_);
        action_t action;
        while(!step.done) {
            action = agent_.policy(step.obs);
            episode.push_back({step, action});

            step = env_.step(agent_, action);
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

    episode_t empty_episode() {
        return {};
    }

private:
    IEnvAgent<DerivedAgent, AgentTraits> agent_;
    IEnv<DerivedEnv, EnvTraits> env_;
    size_t current_{0};
    size_t count_{0};
};


template<typename AgentTraits,
         typename EnvTraits,
         typename DerivedAgent,
         typename DerivedEnv>
struct first_visit_mc_prediction {

    using state_t = typename AgentTraits::state_t;

public:
    first_visit_mc_prediction(IEnvAgent<DerivedAgent, AgentTraits>& agent,
                              IEnv<DerivedEnv, EnvTraits>& env,
                              parameters_set params)
        : agent_(agent), env_(env), params_(params)
    { }

    void operator()() {
        std::unordered_map<state_t, int> N;
        generate_episode episodes_gen(agent_, env_, params_.episodes_count);
        auto episode = episodes_gen.empty_episode();

        while(episodes_gen >> episode) {
            double gain = 0.0;
            int obs_idx = episode.size() - 1;
            for(auto obs_it = episode.rbegin(); obs_it != episode.rend(); ++obs_it, --obs_idx) {
                auto [state, reward, done] = (*obs_it).step;

                gain = params_.discont*gain + reward;
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
    IEnvAgent<DerivedAgent, AgentTraits> agent_;
    IEnv<DerivedEnv, EnvTraits> env_;
    parameters_set params_;
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

template<typename AgentTraits,
         typename EnvTraits,
         typename DerivedAgent,
         typename DerivedEnv>
void first_visit_mc_control(IEnvAgent<DerivedAgent, AgentTraits>& agent,
                            IEnv<DerivedEnv, EnvTraits>& env,
                            parameters_set params)
{
    using state_t = typename AgentTraits::state_t;
    using action_t = typename AgentTraits::action_t;

    std::unordered_map<state_t, std::unordered_map<action_t, int>> n_sa;
    std::unordered_map<state_t, int> n_s;
    generate_episode episodes_gen{agent, env, params.episodes_count};
    auto episode = episodes_gen.empty_episode();

    while(episodes_gen >> episode) {
        double gain{0.0};
        for(auto step_it = episode.rbegin(); step_it != episode.rend(); ++step_it) {
            auto [obs, reward, done] = (*step_it).step;
            state_t state = agent.observe(obs);
            action_t action = (*step_it).action;
            gain = params.discont*gain + reward;
            auto it = first_of(episode, state, action);
            if(std::make_reverse_iterator(++it) == step_it) {
                double nsa = ++n_sa[state][action];
                double ns = ++n_s[state];
                double q = agent.value_action(state, action);
                agent.value_action(state, action) = q + (1.0 / nsa ) * (gain - q);
                agent.update_policy(state, params.exploration_prob(ns));
            }
        }
    }

}

template<typename AgentTraits,
         typename EnvTraits,
         typename DerivedAgent,
         typename DerivedEnv>
void sarsa(IEnvAgent<DerivedAgent, AgentTraits>& agent,
           IEnv<DerivedEnv, EnvTraits>& env,
           double lambda,
           parameters_set params)
{
    using state_t = typename AgentTraits::state_t;
    using action_t = typename AgentTraits::action_t;
    using observation_t = typename AgentTraits::observation_t;
    using step_t = typename IEnv<DerivedEnv, AgentTraits>::step_t;

    std::unordered_map<state_t, int> n_s;
    std::unordered_map<state_t, std::unordered_map<action_t, int>> n_sa;

    for(size_t n = 0; n < params.episodes_count; ++n) {
        std::unordered_map<state_t, std::unordered_map<action_t, double>> e;
        env.reset();
        auto init_step = env.init(agent);
        state_t state1 = agent.observe(init_step.obs), state2;
        action_t action1 = agent.policy(state1), action2;

        while(true) {
            double ns = ++n_s[state1];
            double nsa = ++n_sa[state1][action1];
            auto [obs, reward, done] = env.step(agent, action1);
            state2 = agent.observe(obs); action2 = agent.policy(state2);

            double delta = reward + params.discont*agent.value_action(state2, action2) - agent.value_action(state1, action1);
            e[state1][action1]++;

            agent.for_each([&](state_t state, action_t action){
                agent.value_action(state, action) += params.learning_rate(nsa) * delta * e[state][action];
                e[state][action] *= params.discont * lambda;
            });

            agent.update_policy(state1, params.exploration_prob(ns));
            state1 = state2; action1 = action2;
            if(done) break;
        }

    }

}

template<typename AgentTraits,
         typename EnvTraits,
         typename DerivedAgent,
         typename DerivedEnv,
         typename Approximation>
void sarsa(IApproxAgent<DerivedAgent, AgentTraits, Approximation>& agent,
           IEnv<DerivedEnv, EnvTraits>& env,
           double lambda,
           parameters_set params)
{
    using state_t = typename AgentTraits::state_t;
    using action_t = typename AgentTraits::action_t;
    using observation_t = typename AgentTraits::observation_t;
    using step_t = typename IEnv<DerivedEnv, AgentTraits>::step_t;
    using approx_state_t = typename Approximation::approx_state_t;

    double alpha = 0.01;
    double discont = params.discont;

    for(size_t n = 0; n < params.episodes_count; ++n) {
        Approximation e;
        env.reset();
        auto init_step = env.init(agent);
        state_t state1 = init_step.obs, state2;
        action_t action1 = agent.policy(state1), action2;

        while(true) {
            auto [obs, reward, done] = env.step(agent, action1);
            state2 = obs; action2 = agent.policy(state2);
            double delta = reward + discont * agent.value_action(state2, action2) - agent.value_action(state1, action1);
            e(state1, action1) += 1.0;

            agent.for_each([&](state_t state, action_t action){
                approx_state_t tmp = e(state, action) * alpha * delta;
                agent.value_action(state, action) += tmp;
                e(state, action) *= discont * lambda;
            });

            state1 = state2; action1 = action2;
            if(done) break;
        }

    }
}

} // namespace rl::algorithms


namespace rl {

using rl::algorithms::parameters_set;
using rl::algorithms::create_default_params_set;
using rl::algorithms::generate_episode;

using rl::algorithms::value_iteration;

using rl::algorithms::first_visit_mc_prediction;
using rl::algorithms::first_visit_mc_control;
using rl::algorithms::sarsa;


} // namespace rl


#endif // RL_ALGO_HPP
