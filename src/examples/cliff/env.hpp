#ifndef ENV_HPP
#define ENV_HPP
#pragma once

#include <map>
#include <random>

#include <rl/traits.hpp>
#include <rl/ienv.hpp>

#include "model.hpp"


namespace std {

template<>
struct hash<cliff::model::position> {
    std::size_t operator()(const cliff::model::position& key) const {
        return std::hash<int>{}(key.first) ^ (std::hash<int>{}(key.second) << 1);
    }
};

}


namespace cliff {

using position = model::position;
using action_e = model::action_e;
using env_traits = rl::EnvTraits<position, action_e>;
using reward_t = env_traits::reward_t;

struct Env : rl::IEnv<Env, env_traits> {

    static constexpr double discont { 1.0 };

public:
    std::map<size_t, position> players_position_;
    model::board board_;

    Env(const model::board& board)
        : board_(board)
    { }

    step_t init_impl(size_t agent_id) {
        players_position_[agent_id] = board_.start_position_;

        observation_t state = players_position_[agent_id];
        return std::make_tuple(state, 0.0, false);
    }

    step_t step_impl(size_t agent_id, const action_t& action) {

        position& player_position = players_position_[agent_id];
        reward_t reward = 0, penalty = 10;

        switch (action) {
        case action_e::up: {
            if(player_position.second + 1 < board_.height_)
                ++player_position.second;
            else
                reward -= penalty;

            break;
        }
        case action_e::down: {
            if(player_position.second - 1 >= 0)
                --player_position.second;
            else
                reward -= penalty;

            break;
        }
        case action_e::left: {
            if(player_position.first - 1 >= 0)
                --player_position.first;
            else
                reward -= penalty;

            break;
        }
        case action_e::right: {
            if(player_position.first + 1 < board_.width_)
                ++player_position.first;
            else
                reward -= penalty;

            break;
        }
        }

        bool done = false;

        if(board_.grid_[player_position.second][player_position.first] == model::field_e::hole) {
            done = true;
            reward -= 100;
        }
        else if(board_.finish_position_ != player_position) {
//            if(reward != 0) {
//                done = true;
//                reward -= 100;
//            }
//            else
                reward -= 1;
        }
        else {
            done = true;
            reward += 1;
        }

        return std::make_tuple(player_position, reward, done);
    }

    void reset_impl() {
        for(auto& [key, value]: players_position_)
            players_position_[key] = board_.start_position_;
    }
};

} // namespace blackjack

#endif // ENV_HPP
