#include <iostream>

#include <utility.hpp>
#include <tasks/coin/task.hpp>
#include <rl_algo.hpp>

#include <tasks/blackjack/env.hpp>

using namespace std;

int main()
{
    blackjack::Agent agent;
    blackjack::Env env;
    rl::first_visit_mc_control(agent, env, 0.05, 100000);

    rl::first_visit_mc_prediction(agent, env, 100);

//    blackjack::Env env;
//    using traits = blackjack::Env::traits_t;
//    auto policy = [](traits::state_t state) -> traits::action_t {
//        return state.first < 12 ? traits::action_t::hit : traits::action_t::stick;
//    };
//    rl::generate_episode{env, policy}();
    
    return 0;
}
