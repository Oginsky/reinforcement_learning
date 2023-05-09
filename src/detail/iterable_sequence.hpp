#ifndef INDEXES_SEQUENCE_HPP
#define INDEXES_SEQUENCE_HPP
#pragma once

#include <iterator>


template <typename Value>
struct iterable_sequence {

    template <typename T>
    struct sequence_iterator {

    public:
        using iterator_category = std::forward_iterator_tag;
        using value_t = T;
        using reference_t = value_t&;
        using pointer_t = value_t*;

    public:
        sequence_iterator(value_t value)
            : value_(value) { }

        reference_t operator*() { return value_; }
        pointer_t operator->() { return &value_; }
        sequence_iterator operator++() { ++value_; return *this; }
        sequence_iterator operator++(int) { sequence_iterator tmp = *this; ++(*this); return tmp; }
        bool operator!=(const sequence_iterator& rhs) { return value_ != rhs.value_; }
        bool operator==(const sequence_iterator& rhs) { return value_ == rhs.value_; }

    private:
        value_t value_;
    };


public:
    using value_t = Value;
    using iterator_t = sequence_iterator<Value>;
    using const_iterator_t = const sequence_iterator<Value>;

    iterable_sequence(value_t start, value_t end)
        : start_(start), end_(end)
    {

    }

    iterator_t begin() {
        return iterator_t(start_);
    }

    const_iterator_t begin() const {
        return const_iterator_t(start_);
    }

    iterator_t end() {
        return iterator_t(end_ + 1);
    }

    const_iterator_t end() const {
        return const_iterator_t(end_ + 1);
    }

private:
    value_t start_, end_;
};

#endif // INDEXES_SEQUENCE_HPP
