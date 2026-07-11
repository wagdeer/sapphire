#pragma once

#include <array>
#include <cstddef>
#include <iterator>

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
        return data_[physicalIndex(i)];
    }
    const T& operator[](size_type i) const {
        return data_[physicalIndex(i)];
    }
    T& front() { return (*this)[0]; }
    const T& front() const { return (*this)[0]; }
    T& back() { return (*this)[size_ - 1]; }
    const T& back() const { return (*this)[size_ - 1]; }

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
    class iterator {
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        iterator(RingBuffer* buf, size_type idx) : buf_(buf), idx_(idx) {}
        reference operator*() const { return (*buf_)[idx_]; }
        pointer operator->() const { return &(*buf_)[idx_]; }
        iterator& operator++() { ++idx_; return *this; }
        iterator operator++(int) { auto t = *this; ++idx_; return t; }
        iterator& operator--() { --idx_; return *this; }
        iterator operator--(int) { auto t = *this; --idx_; return t; }
        iterator& operator+=(difference_type n) { idx_ += n; return *this; }
        iterator& operator-=(difference_type n) { idx_ -= n; return *this; }
        iterator operator+(difference_type n) const { return {buf_, idx_ + n}; }
        iterator operator-(difference_type n) const { return {buf_, idx_ - n}; }
        difference_type operator-(const iterator& other) const { return idx_ - other.idx_; }
        reference operator[](difference_type n) const { return (*buf_)[idx_ + n]; }
        bool operator==(const iterator& other) const { return idx_ == other.idx_; }
        bool operator!=(const iterator& other) const { return idx_ != other.idx_; }
        bool operator<(const iterator& other) const { return idx_ < other.idx_; }
    private:
        RingBuffer* buf_;
        size_type idx_;
    };

    using reverse_iterator = std::reverse_iterator<iterator>;

    iterator begin() { return {this, 0}; }
    iterator end()   { return {this, size_}; }
    reverse_iterator rbegin() { return reverse_iterator(end()); }
    reverse_iterator rend()   { return reverse_iterator(begin()); }

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
