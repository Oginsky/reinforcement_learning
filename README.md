# Reinforcement Learning (`rl::`)
A simple header only template library for reinforcement learning algorithms

## Dependencies
+ С++17

No third-party dependencies, just include the header files

## Features
+ Easy to use
+ Header-only
+ Static polymorphism
+ Customizable agent types
+ Single interface

## Implemented
| method                            | Category               |
| --------------------------------- | ---------------------- |
| `rl::value_iteration`             | Strategy improvement   |
| `rl::first_visit_mc_prediction`   | strategy evaluation    |
| `rl::first_visit_mc_control`      | Strategy improvement   |
| `rl::sarsa`                       | Strategy improvement   |

## Usage
1. Include a header file

All you have to do is include a single header file:
```cpp
#include <rl/rl.hpp>
```

2. Type traits

Types for representing the interaction of an agent with the environment are described in the type traits structure:
```cpp
template <typename Action,              // Actions of an agent with the environment  
          typename State,               // Agent state
          typename Reward = double,     // The reward returned to the agent by the environment is usually just a number
          typename Observation = State> // The type of observation, usually coincides with the state of the agent
struct Traits {
    using action_t = Action;
    using state_t = State;
    using reward_t = Reward;
    using observation_t = Observation;
};

```

3. Enviroment implementation

The `rl::IEnv` interface is provided for enviroment:
```cpp
template <typename Derived, typename Traits>
struct IEnv {
    using traits_t = Traits;
    using action_t = typename traits_t::action_t;
    using state_t  = typename traits_t::state_t;
    using reward_t = typename traits_t::reward_t;
    using observation_t = typename traits_t::observation_t;
    using step_tuple_t = std::tuple<observation_t, reward_t, bool>;

public:
    IEnv()
    virtual ~IEnv()

    step_t init() 

    step_t step(const state_t& state, const action_t& action)

    void reset() 
};

```
It is necessary to inherit this class and implement the corresponding methods with suffix **_impl**.

4. Agent implementation

The `rl::IEnvAgent` interface is provided for agents that interact with the environment:
```cpp
template <typename Derived, typename Traits>
struct IEnvAgent : public IAgent<Derived, Traits>
{
    using traits_t = typename IAgent<Derived, Traits>::traits_t;
    using action_t = typename traits_t::action_t;
    using state_t = typename traits_t::state_t;
    using observation_t = typename traits_t::observation_t;
public:
    IEnvAgent(double eps = 0.0)
    virtual ~IEnvAgent()

    double& value_action(state_t state, action_t action) 
    
    action_t get_best_action(state_t state)
    
    state_t get_state(observation_t observation)
    
    action_t policy(observation_t observation) 
    
    void update_policy(state_t state, double eps) 
};
```
It is necessary to inherit this class and implement the corresponding methods with suffix **_impl**, because in the interface class the implementation looks like this:
```cpp
action_t get_best_action(state_t state) {
    return derived->get_best_action_impl(state);
}
```

## Examples of use
Examples of practical use of the library can be found in [`src/example`](src/examples):

| Task                | Description                                                                                       | Categoty                          |
| ------------------- | ------------------------------------------------------------------------------------------------- | --------------------------------- |
| `blackjack_mini`    | Blackjack" card game with simplified rules                                                        | Based on environment interaction  | 
| `coin`              | Example 4.3. Reinforcement Learning: An Introduction. Richard S. Sutton and Andrew G. Barto.      | Based on mdp                      |

*Example of a graph function for the value of an agent's state in the game blackjack_mimi:*
![state function graph](https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/value_function_500k_episodes.png)
