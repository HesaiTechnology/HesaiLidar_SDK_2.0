#include "ring_2dex.h"
#include <cassert>
#include <iterator>
using namespace hesai::lidar;
template <typename T, size_t N, typename T2, size_t M>
template <typename Item>
class Ring2D_ex<T, N, T2, M>::ItemIterator {
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;
private:
    pointer _p0;
    difference_type _steps;
public:
    ItemIterator(const ItemIterator&) = default;
    ItemIterator& operator=(const ItemIterator&) = default;
    ItemIterator(pointer p0, difference_type steps) : _p0(p0), _steps(steps) {}

    inline ItemIterator& operator++() { ++_steps; return *this; }
    inline ItemIterator& operator--() { assert(_steps >= 1); --_steps; return *this; }
    inline ItemIterator operator++(int) { ItemIterator old(*this); ++(*this); return old; }
    inline ItemIterator operator--(int) { assert(_steps >= 1); ItemIterator old(*this); --(*this); return old; }
    inline ItemIterator operator+(difference_type n) const { return ItemIterator(_p0, _steps + n); }
    inline ItemIterator operator-(difference_type n) const { assert(_steps >= n); return ItemIterator(_p0, _steps - n); }
    inline ItemIterator& operator+=(difference_type n) { _steps += n; return *this; }
    inline ItemIterator& operator-=(difference_type n) { assert(_steps >= n); _steps -= n; return *this; }
    inline reference operator*() const { return _p0[_steps % N]; }
    inline operator pointer () const { return _p0 + _steps % N; }
    inline pointer p0() const { return _p0; }
    inline difference_type steps() const { return _steps; }
    inline bool operator==(const ItemIterator& other) const { return this->p0() == other.p0() && this->steps() == other.steps(); }
    inline bool operator!=(const ItemIterator& other) const { return !(*this == other); }
    inline difference_type operator-(const ItemIterator& other) const { assert(this->p0() == other.p0()); return this->steps() - other.steps(); }
};

template <typename T, size_t N, typename T2, size_t M>
Ring2D_ex<T, N, T2, M>::Ring2D_ex() : _ring(new std::array<T, N>), _ring2(new std::array<T2, N*M>), _begin(0), _size(0) {};

template <typename T, size_t N, typename T2, size_t M>
inline constexpr size_t Ring2D_ex<T, N, T2, M>::size() const { return _size; }

template <typename T, size_t N, typename T2, size_t M>
inline bool Ring2D_ex<T, N, T2, M>::empty() const { return _size == 0; }

template <typename T, size_t N, typename T2, size_t M>
inline bool Ring2D_ex<T, N, T2, M>::not_empty() const { return !empty(); }

template <typename T, size_t N, typename T2, size_t M>
inline bool Ring2D_ex<T, N, T2, M>::full() const { return _size >= N; }

template <typename T, size_t N, typename T2, size_t M>
inline bool Ring2D_ex<T, N, T2, M>::not_full() const { return !full(); }

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::clear() { for (auto& item : *this) { item = T(); } _size = 0; _begin = 0; }

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::eff_clear() { _size = 0; _begin = 0; }

template <typename T, size_t N, typename T2, size_t M>
template <typename... Args>
inline void Ring2D_ex<T, N, T2, M>::emplace_back(Args&&... args) { assert(!full()); (*_ring)[(_begin + _size) % N] = T(args...); ++_size;}

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::push_back(T&& item) { assert(!full()); (*_ring)[(_begin + _size) % N] = item; ++_size; }

template <typename T, size_t N, typename T2, size_t M>
inline const T& Ring2D_ex<T, N, T2, M>::peek_back() const { assert(!empty()); return (*_ring)[(_begin + _size - 1) % N]; }

template <typename T, size_t N, typename T2, size_t M>
inline T Ring2D_ex<T, N, T2, M>::pop_back() { assert(!empty()); --_size; T item; std::swap(item, (*_ring)[(_begin + _size) % N]); return item; }

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::eff_pop_back() { --_size; }

template <typename T, size_t N, typename T2, size_t M>
template <typename... Args>
inline void Ring2D_ex<T, N, T2, M>::emplace_front(Args&&... args) { assert(!full()); _begin = (_begin + N - 1) % N; (*_ring)[_begin] = T(args...); ++_size; }

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::push_front(T&& item) { assert(!full()); _begin = (_begin + N - 1) % N; (*_ring)[_begin] = item; ++_size; }

template <typename T, size_t N, typename T2, size_t M>
inline const T& Ring2D_ex<T, N, T2, M>::peek_front() const { assert(!empty()); return (*_ring)[_begin]; }

template <typename T, size_t N, typename T2, size_t M>
inline T Ring2D_ex<T, N, T2, M>::pop_front() { assert(!empty()); --_size; T item; std::swap(item, (*_ring)[_begin]); _begin = (_begin + 1) % N; return item; }

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::eff_pop_front() { --_size; _begin = (_begin + 1) % N; }

template <typename T, size_t N, typename T2, size_t M>
inline typename Ring2D_ex<T, N, T2, M>::iterator Ring2D_ex<T, N, T2, M>::begin() { return iterator((T*)_ring.get(), _begin); }

template <typename T, size_t N, typename T2, size_t M>
inline typename Ring2D_ex<T, N, T2, M>::iterator Ring2D_ex<T, N, T2, M>::end() { return iterator((T*)_ring.get(), _begin + _size); }

template <typename T, size_t N, typename T2, size_t M>
inline typename Ring2D_ex<T, N, T2, M>::const_iterator Ring2D_ex<T, N, T2, M>::cbegin() const { return const_iterator((const T*)_ring.get(), _begin); }

template <typename T, size_t N, typename T2, size_t M>
inline typename Ring2D_ex<T, N, T2, M>::const_iterator Ring2D_ex<T, N, T2, M>::cend() const { return const_iterator((const T*)_ring.get(), _begin + _size); }

template <typename T, size_t N, typename T2, size_t M>
inline T& Ring2D_ex<T, N, T2, M>::operator[](size_t index) { assert(index < size()); return (*_ring)[(_begin + index) % N]; }

template <typename T, size_t N, typename T2, size_t M>
inline const T& Ring2D_ex<T, N, T2, M>::operator[](size_t index) const { assert(index < size()); return (*_ring)[(_begin + index) % N]; }

template <typename T, size_t N, typename T2, size_t M>
inline T* Ring2D_ex<T, N, T2, M>::data() { return _ring->data(); }

template <typename T, size_t N, typename T2, size_t M>
inline const T* Ring2D_ex<T, N, T2, M>::data() const { return _ring->data(); };

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::prepare_emplace_back(size_t s, size_t t) 
{
    assert(s <= N && t <= M);
    while (_size + s > N) 
    {
        eff_pop_front();
    }
    _emplace_start = (_begin + _size) % N;
    _emplace_size = s;
    _2D_size = t;
};

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::emplace_back_at(size_t i, T&& item) 
{
    assert(i < _emplace_size);
    (*_ring)[(_emplace_start + i) % N] = item;
};

template <typename T, size_t N, typename T2, size_t M>
inline T& Ring2D_ex<T, N, T2, M>::peek_back_at(size_t i) 
{
    assert(i < _emplace_size);
    return (*_ring)[(_emplace_start + i) % N];
};

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::emplace_back_at(size_t i, size_t j, T2&& item) 
{
    assert((i < _emplace_size) && (j < _2D_size));
    (*_ring2)[((_emplace_start + i) % N) * _2D_size + j] = item;
};

template <typename T, size_t N, typename T2, size_t M>
inline T2& Ring2D_ex<T, N, T2, M>::peek_back_at(size_t i, size_t j) 
{
    assert((i < _emplace_size) && (j < _2D_size));
    return (*_ring2)[((_emplace_start + i) % N) * _2D_size + j];
};

template <typename T, size_t N, typename T2, size_t M>
inline void Ring2D_ex<T, N, T2, M>::finish_emplace_back() 
{
    _size += _emplace_size;
};

template <typename T, size_t N, typename T2, size_t M>
inline T2* Ring2D_ex<T, N, T2, M>::data2() { return _ring2->data(); }

template <typename T, size_t N, typename T2, size_t M>
inline const T2* Ring2D_ex<T, N, T2, M>::data2() const { return _ring2->data(); };

template <typename T, size_t N, typename T2, size_t M>
inline T2& Ring2D_ex<T, N, T2, M>::peek_front(size_t j) const { 
    assert(!empty() && j < _2D_size); 
    return (*_ring2)[_begin * _2D_size + j]; 
}

