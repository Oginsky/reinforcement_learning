#include <iostream>
#include <fstream>

#include <rl/utility.hpp>
#include <rl/algorithm.hpp>

#include "env.hpp"
#include "agent.hpp"


using namespace std;


int main() {
    // types allias
    using state_t = blackjack::Agent::state_t;
    using action_t = blackjack::Agent::action_t;
    using dict_t = std::map<state_t, std::map<action_t, double>>;

    blackjack::Env env;

    // calc the true value of the agent state function
    blackjack::Agent mc_agent;
    rl::first_visit_mc_control(mc_agent, env, 0.05, 500000);
    converse_value_action(mc_agent.value_action, mc_agent.value_func);
    write_in_file("blackjack_value_function.txt", mc_agent.value_func);

    // func of calc standart error value
    auto mse = [](dict_t& true_val, dict_t& appr_val) {
        double result{0.0};
        for(size_t player_points = 1; player_points < 23; ++player_points) {
            for(size_t dealer_points = 1; dealer_points < 12; ++dealer_points) {
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

    // dependence of the standart error on parameter lambda for 1000 episodes
    std::ofstream fout("mse_on_lambda.txt");
    blackjack::Agent sarsa_agent;
    for(int i = 10; i <= 10; ++i) {
        sarsa_agent.reinit();
        env.reset();

        double lambda = 0.1 * i;
        rl::sarsa(sarsa_agent, env, lambda, 1000);
        double mse_val = mse(mc_agent.value_action, sarsa_agent.value_action);
        fout << lambda << " " << mse_val << std::endl;
    }
    fout.close();

    // calc of the learning curve
    // dependence of the standart error on the episode number
    fout.open("learning_curve_lambda=1.txt");
    double lambda = 1.0;
    for(size_t n = 1000; n < 1000; ++n) {
        sarsa_agent.reinit();
        env.reset();

        rl::sarsa(sarsa_agent, env, lambda, n);
        double mse_val = mse(mc_agent.value_action, sarsa_agent.value_action);
        fout << n << " " << mse_val << std::endl;
    }

    return 0;
}
