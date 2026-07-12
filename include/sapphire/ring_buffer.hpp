#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>

namespace sapphire {

/// Fixed-capacity ring buffer with random access and reverse iteration.
///
/// Zero heap allocation after construction. O(1) push_back, O(1) random
/// access. When full, new elements silently overwrite the oldest.
///
/// Usage:
///   RingBuffer<ImuData, 500> buf;
///   buf.push_back(imu);          // producer thread
///   for (auto it = buf.rbegin(); it != buf.rend(); ++it) { ... }  // consumer
template <typename T, std::size_t N>
class RingBuffer {
public:
    static_assert(N > 0, "RingBuffer capacity must be greater than zero");

    using value_type = T;
    using size_type = std::size_t;

    RingBuffer() = default;

    // ── Capacity ──────────────────────────────────────────────────
    static constexpr size_type capacity() { return N; }
    size_type size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == N; }

    // ── Element access ────────────────────────────────────────────
    // Index 0 = oldest, size()-1 = newest
    T& operator[](size_type i) {
        assert(i < size_);
        return data_[physicalIndex(i)];
    }
    const T& operator[](size_type i) const {
        assert(i < size_);
        return data_[physicalIndex(i)];
    }
    T& front() {
        assert(!empty());
        return (*this)[0];
    }
    const T& front() const {
        assert(!empty());
        return (*this)[0];
    }
    T& back() {
        assert(!empty());
        return (*this)[size_ - 1];
    }
    const T& back() const {
        assert(!empty());
        return (*this)[size_ - 1];
    }

    // ── Mutation ──────────────────────────────────────────────────
    void push_back(const T& val) {
        data_[head_] = val;
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }
    void push_back(T&& val) {
        data_[head_] = std::move(val);
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }
    void clear() { head_ = 0; size_ = 0; }

    // ── Iterators ─────────────────────────────────────────────────
    // Forward: oldest → newest
    template <bool IsConst>
    class basic_iterator {
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = std::conditional_t<IsConst, const T*, T*>;
        using reference = std::conditional_t<IsConst, const T&, T&>;
        using buffer_type =
            std::conditional_t<IsConst, const RingBuffer, RingBuffer>;

        basic_iterator(buffer_type* buf, size_type idx)
            : buf_(buf), idx_(idx) {}
        reference operator*() const { return (*buf_)[idx_]; }
        pointer operator->() const { return &(*buf_)[idx_]; }
        basic_iterator& operator++() { ++idx_; return *this; }
        basic_iterator operator++(int) { auto t = *this; ++idx_; return t; }
        basic_iterator& operator--() { --idx_; return *this; }
        basic_iterator operator--(int) { auto t = *this; --idx_; return t; }
        basic_iterator& operator+=(difference_type n) {
            idx_ = static_cast<size_type>(
                static_cast<difference_type>(idx_) + n);
            return *this;
        }
        basic_iterator& operator-=(difference_type n) { return *this += -n; }
        basic_iterator operator+(difference_type n) const {
            auto result = *this;
            result += n;
            return result;
        }
        basic_iterator operator-(difference_type n) const {
            auto result = *this;
            result -= n;
            return result;
        }
        difference_type operator-(const basic_iterator& other) const {
            return static_cast<difference_type>(idx_)
                - static_cast<difference_type>(other.idx_);
        }
        reference operator[](difference_type n) const { return (*buf_)[idx_ + n]; }
        bool operator==(const basic_iterator& other) const {
            return buf_ == other.buf_ && idx_ == other.idx_;
        }
        bool operator!=(const basic_iterator& other) const {
            return !(*this == other);
        }
        bool operator<(const basic_iterator& other) const {
            return idx_ < other.idx_;
        }
        bool operator>(const basic_iterator& other) const {
            return other < *this;
        }
        bool operator<=(const basic_iterator& other) const {
            return !(other < *this);
        }
        bool operator>=(const basic_iterator& other) const {
            return !(*this < other);
        }
        friend basic_iterator operator+(
            difference_type n, const basic_iterator& iterator) {
            return iterator + n;
        }
    private:
        buffer_type* buf_;
        size_type idx_;
    };

    using iterator = basic_iterator<false>;
    using const_iterator = basic_iterator<true>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    iterator begin() { return {this, 0}; }
    iterator end()   { return {this, size_}; }
    const_iterator begin() const { return {this, 0}; }
    const_iterator end() const { return {this, size_}; }
    const_iterator cbegin() const { return begin(); }
    const_iterator cend() const { return end(); }
    reverse_iterator rbegin() { return reverse_iterator(end()); }
    reverse_iterator rend()   { return reverse_iterator(begin()); }
    const_reverse_iterator rbegin() const {
        return const_reverse_iterator(end());
    }
    const_reverse_iterator rend() const {
        return const_reverse_iterator(begin());
    }
    const_reverse_iterator crbegin() const { return rbegin(); }
    const_reverse_iterator crend() const { return rend(); }

private:
    size_type physicalIndex(size_type logical) const {
        // Map logical index (0 = oldest) to physical offset in data_
        if (size_ < N) return logical;  // buffer not yet wrapped
        return (head_ + logical) % N;
    }

    std::array<T, N> data_{};
    size_type head_ = 0;  // next write position
    size_type size_ = 0;
};

}  // namespace sapphire
