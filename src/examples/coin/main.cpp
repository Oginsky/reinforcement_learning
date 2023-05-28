#include <iostream>

#include <rl/algorithm.hpp>
#include <rl/utility.hpp>

#include "task.hpp"


using namespace std;


int main() {
    Task task(0.4);
    rl::value_iteration{}(task.agent, task);

    write_in_file("value_function.txt", task.agent.v);
    write_in_file("policy.txt", task.agent.policy_);
	
	return 0;
}
