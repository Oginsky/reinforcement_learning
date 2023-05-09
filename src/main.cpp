#include <iostream>

#include <utility.hpp>
#include <tasks/coin/task.hpp>
#include <rl_algo.hpp>


using namespace std;

int main()
{
    Task task(0.55);
    rl::value_iteration{}(task.agent, task);

    write_in_file("value_function.txt", task.agent.v);
    write_in_file("policy.txt", task.agent.policy_);
    
    return 0;
}
