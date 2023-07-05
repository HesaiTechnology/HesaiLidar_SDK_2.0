#ifndef ___HESAI__CONTAINER__RING2D_EX_HH___
#define ___HESAI__CONTAINER__RING2D_EX_HH___

#include <memory>
#include <array>
namespace hesai
{
namespace lidar
{
template <typename T, size_t N, typename T2, size_t M>
class Ring2D_ex {
public:
    template <typename Item> class ItemIterator;
    using iterator = ItemIterator<T>;
    using const_iterator = ItemIterator<const T>;
private:
    std::unique_ptr<std::array<T, N>> _ring;
    std::unique_ptr<std::array<T2, N * M>> _ring2;
    size_t _begin, _size;
    size_t _emplace_start, _emplace_size;
    size_t _2D_size;
public:
    Ring2D_ex();
    inline constexpr size_t size() const;
    inline bool empty() const;
    inline bool not_empty() const;
    inline bool full() const;
    inline bool not_full() const;
    inline void clear();
    inline void eff_clear();
    template <typename... Args>
    inline void emplace_back(Args&&... args);
    inline void push_back(T&& item);
    inline const T& peek_back() const;
    inline T pop_back();
    inline void eff_pop_back();
    template <typename... Args>
    inline void emplace_front(Args&&... args);
    inline void push_front(T&& item);
    inline const T& peek_front() const;
    inline T pop_front();
    inline void eff_pop_front();
    inline iterator begin();
    inline iterator end();
    inline const_iterator cbegin() const;
    inline const_iterator cend() const;
    inline T& operator[](size_t index);
    inline const T& operator[](size_t index) const;
    inline T* data();
    inline const T* data() const;
    inline void prepare_emplace_back(size_t s, size_t t);
    inline void emplace_back_at(size_t i, T&& item);
    inline T& peek_back_at(size_t i);
    inline void emplace_back_at(size_t i, size_t j, T2&& item);
    inline T2& peek_back_at(size_t i, size_t j);
    inline void finish_emplace_back();
    inline T2* data2();
    inline const T2* data2() const;
    inline T2& peek_front(size_t j) const;
};
}  // namespace lidar
}  // namespace hesai

#endif // !___HESAI__CONTAINER__RING2D_EX_HH___
