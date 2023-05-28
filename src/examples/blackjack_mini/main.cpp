#include <iostream>

#include <rl/utility.hpp>
#include <rl/algorithm.hpp>

#include "env.hpp"


using namespace std;


int main()
{
    blackjack::task();
    blackjack::Agent agent;
    blackjack::Env env;
    rl::first_visit_mc_control(agent, env, 0.05, 250000);
    //rl::sarsa(agent, env, 0.5, 10000);

    converse_value_action(agent.value_action, agent.value_func);

    write_in_file("blackjack_value_function.txt", agent.value_func);

    return 0;
}
