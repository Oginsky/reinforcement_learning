#include <agent.hpp>
#include <env.hpp>
#include <model.hpp>

#include <rl/algorithm.hpp>
#include <rl/utility.hpp>

using namespace cliff;

void save_policy(Agent::container_t& policy, Env& env)
{
    std::ofstream fout("cliff_policy.txt");
    if(!fout.is_open()) return;

    // size
    fout << env.board_.width_ << " " << env.board_.height_ << " " << "\n";

    // data
    for(int y = 0; y < env.board_.height_; ++y) {
        for(int x = 0; x < env.board_.width_; ++x) {

            char direction = 'h';

            if(env.board_.grid_.at(y).at(x) != model::field_e::hole) {
                model::position pos {x, y};

                if(pos == env.board_.finish_position_)
                    direction = 'f';
                else {
                    model::action_e best_action;
                    double max_value = policy[pos].begin()->second;

                    for(auto it = policy[pos].begin(); it != policy[pos].end(); ++it) {
                        if(it->second >= max_value) {
                            best_action = it->first;
                            max_value = it->second;
                        }
                    }

                    switch (best_action) {
                    case action_e::up: {
                        direction = 'u';
                        break;
                    }
                    case action_e::down: {
                        direction = 'd';
                        break;
                    }
                    case action_e::left: {
                        direction = 'l';
                        break;
                    }
                    case action_e::right: {
                        direction = 'r';
                        break;
                    }
                    }
                }
            }

            fout << x << " " << y << " " << direction << "\n";
        }
    }
}

int main()
{
    int w = 10, h = 10;
    model::board board(w, h);

    // fill board
    {
        size_t y = 0;
        size_t x = board.width_ - 1;

        board.start_position_ = { 0, y };
        board.finish_position_ = { x, y };

        for(int i = 1; i < board.width_ - 1; ++i) {
            board.grid_.at(y).at(i) = model::field_e::hole;
        }

        for(int i = 2; i < board.width_ - 2; ++i) {
            board.grid_.at(y + 1).at(i) = model::field_e::hole;
        }

        for(int i = 1; i < board.width_ - 3; ++i) {
            board.grid_.at(y + 2).at(i) = model::field_e::hole;
        }

        for(int i = 0; i < board.height_ - 1; ++i) {
            board.grid_.at(i).at(1) = model::field_e::hole;
        }

        for(int i = 0; i < board.height_ - 1; ++i) {
            board.grid_.at(i).at(board.width_ - 2) = model::field_e::hole;
        }

        board.grid_.at(2).at(board.width_ - 2) = model::field_e::passage;

        board.grid_.at(board.height_ / 2).at(1) = model::field_e::passage;
    }

    Env env(board);
    Agent agent(board.width_, board.height_);

    rl::parameters_set params(1.0,
        500'000,
        [](int n){return 1.0 / (double)n;},
        [](int n){return 100.0 / (100.0 + n);});

    rl::q_learning(agent, env, params);
//    rl::sarsa(agent, env, 0.25, params);

    save_policy(agent.policy_, env);

    return 0;
}
