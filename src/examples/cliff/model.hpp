#ifndef MODEL_HPP
#define MODEL_HPP

#include <vector>


namespace cliff::model {

enum class action_e { up, down, left, right };

using position = std::pair<int, int>;

enum class field_e { passage, hole };

struct board {

    int width_;
    int height_;

    std::vector<std::vector<field_e>> grid_;

    position start_position_;
    position finish_position_;

    board() = delete;

    board(std::size_t w, std::size_t h)
        : width_(w)
        , height_(h)
    {
        grid_ = std::vector(height_, std::vector(width_, field_e::passage));
    }

    board(const board& other)
        : width_(other.width_)
        , height_(other.height_)
        , grid_(other.grid_)
        , start_position_(other.start_position_)
        , finish_position_(other.finish_position_)
    { }
};

}

#endif // MODEL_HPP
