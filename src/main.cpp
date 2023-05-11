#include <iostream>

#include <utility.hpp>
#include <tasks/coin/task.hpp>
#include <rl_algo.hpp>

#include <tasks/blackjack/env.hpp>

using namespace std;

int main()
{
    Task task(0.55);
    rl::value_iteration{}(task.agent, task);

    write_in_file("value_function.txt", task.agent.v);
    write_in_file("policy.txt", task.agent.policy_);


//    blackjack::Env env;
//    using traits = blackjack::Env::traits_t;
//    auto policy = [](traits::state_t state) -> traits::action_t {
//        return state.first < 12 ? traits::action_t::hit : traits::action_t::stick;
//    };
//    rl::generate_episode{env, policy}();
    
    return 0;
}
