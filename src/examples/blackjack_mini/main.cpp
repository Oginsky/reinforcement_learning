#include <iostream>
#include <fstream>

#include <rl/utility.hpp>
#include <rl/algorithm.hpp>

#include "env.hpp"
#include "agent.hpp"
#include "linear_agent.hpp"


using namespace std;

// types allias
using state_t = blackjack::Agent::state_t;
using action_t = blackjack::Agent::action_t;
using dict_t = std::map<state_t, std::map<action_t, double>>;

// func of calc standart error value
auto mse = [](dict_t& true_val, dict_t& appr_val) {
    double result{0.0};
    for(size_t player_points = 1; player_points < 21; ++player_points) {
        for(size_t dealer_points = 1; dealer_points < 10; ++dealer_points) {
            state_t state{player_points, dealer_points};
            auto& action_map = true_val[state];

            for(auto& val: action_map) {
                double dx = val.second - appr_val[state][val.first];
                result += dx*dx;
            }
        }
    }

    return result;
};

void sarsa_statistic(blackjack::Env& env, dict_t& value_action) {
    // dependence of the standart error on parameter lambda for 10000 episodes
    std::ofstream fout("mse_on_lambda.txt");
    blackjack::Agent sarsa_agent;
    for(int i = 0; i <= 10; ++i) {
        sarsa_agent.reinit();
        env.reset();

        double lambda = 0.1 * i;
        rl::sarsa(sarsa_agent, env, lambda, 10000);
        double mse_val = mse(value_action, sarsa_agent.value_action);
        fout << lambda << " " << mse_val << std::endl;
    }
    fout.close();
}

void linear_sarsa_statistics(blackjack::Env& env, dict_t& value_action) {
    // dependence of the standart error on parameter lambda for 10000 episodes
    // with linear approximation of the state function
    std::ofstream fout("linear_mse_on_lambda.txt");
    blackjack::LinearAgent linear_agent(0.05);
    for(int i = 0; i <= 10; ++i) {
        linear_agent.reinit();
        env.reset();

        double lambda = 0.1 * i;
        rl::sarsa(linear_agent, env, lambda, 10000);

        dict_t q;
        std::map<state_t, double> value_func;
        // reconstruction agent's value action function
        for(size_t player_points = 1; player_points < 21; ++player_points) {
            for(size_t dealer_points = 1; dealer_points < 10; ++dealer_points) {
                state_t state{player_points, dealer_points};
                q[state][action_t::hit] = linear_agent.value_action_impl(state, action_t::hit);
                q[state][action_t::stick] = linear_agent.value_action_impl(state, action_t::stick);
            }
        }

        double mse_val = mse(value_action, q);
        fout << lambda << " " << mse_val << std::endl;
    }
    fout.close();
}


int main() {
    blackjack::Env env;

    // calc the true value of the agent state function
    blackjack::Agent mc_agent;
    rl::first_visit_mc_control(mc_agent, env, 0.05, 500'000);
    converse_value_action(mc_agent.value_action, mc_agent.value_func);
    write_in_file("blackjack_value_function.txt", mc_agent.value_func);

    // Agent without approximation
    sarsa_statistic(env, mc_agent.value_action);

    // Agent with linear approximation of the state function
    linear_sarsa_statistics(env, mc_agent.value_action);

    return 0;
}
