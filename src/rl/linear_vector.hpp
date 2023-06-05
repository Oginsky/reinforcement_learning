#ifndef LINEAR_VECTOR_HPP
#define LINEAR_VECTOR_HPP
#pragma once

#include <vector>

namespace rl {


template <size_t size, class Source>
struct linear_vector {
    using feature_vec_t = std::vector<bool>;
    using source_t = Source;

public:
    linear_vector() = delete;
    linear_vector(const linear_vector&) = default;
    linear_vector(linear_vector&&) = default;

    explicit linear_vector(source_t& source)
        : feature_vec_(false, size), source_(source) { }

    explicit linear_vector(const feature_vec_t& featur_vec, source_t& source)
        : feature_vec_(featur_vec), source_(source) { }

    ~linear_vector() = default;


public:
    linear_vector operator*(double scalar) {
        for(size_t i = 0; i < size; ++i)
            if(feature_vec_[i])
                source_[i] *= scalar;

        return *this;
    }

    operator double() const {
        double scalar = 0.0;
        for(size_t i = 0; i < size; ++i)
            if(feature_vec_[i])
                scalar += source_[i];

        return scalar;
    }

    linear_vector& operator+=(double scalar) {
        for(size_t i =0; i < size; ++i)
            if(feature_vec_[i])
                source_[i] += scalar;

        return *this;
    }

    linear_vector& operator*=(double scalar) {
        for(size_t i =0; i < size; ++i)
            if(feature_vec_[i])
                source_[i] *= scalar;

        return *this;
    }

    linear_vector& operator+=(const linear_vector& rhs) {
        for(size_t i =0; i < size; ++i)
            if(feature_vec_[i])
                source_[i] += rhs.source_[i];
    }


private:
    feature_vec_t feature_vec_;
    source_t& source_;
};


} // namespace rl

#endif // LINEAR_VECTOR_HPP
