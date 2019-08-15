//
// Created by sigi on 30.09.18.
//

#ifndef EPIPHANY_U_H
#define EPIPHANY_U_H

#include <ostream>
#include <iostream>
#include <functional>
#include <future>
#include <vector>
#include <list>
#include <algorithm>

#ifndef _MSC_VER
#  if __cplusplus < 201103
#    define CONSTEXPR11_TN
#    define CONSTEXPR14_TN
#    define NOEXCEPT_TN
#  elif __cplusplus < 201402
#    define CONSTEXPR11_TN constexpr
#    define CONSTEXPR14_TN
#    define NOEXCEPT_TN noexcept
#  else
#    define CONSTEXPR11_TN constexpr
#    define CONSTEXPR14_TN constexpr
#    define NOEXCEPT_TN noexcept
#  endif
#else  // _MSC_VER
#  if _MSC_VER < 1900
#    define CONSTEXPR11_TN
#    define CONSTEXPR14_TN
#    define NOEXCEPT_TN
#  elif _MSC_VER < 2000
#    define CONSTEXPR11_TN constexpr
#    define CONSTEXPR14_TN
#    define NOEXCEPT_TN noexcept
#  else
#    define CONSTEXPR11_TN constexpr
#    define CONSTEXPR14_TN constexpr
#    define NOEXCEPT_TN noexcept
#  endif
#endif  // _MSC_VER
namespace U {
    class static_string
    {
        const char* const p_;
        const std::size_t sz_;

    public:
        typedef const char* const_iterator;

        template <std::size_t N>
        CONSTEXPR11_TN static_string(const char(&a)[N]) NOEXCEPT_TN
                : p_(a)
                , sz_(N-1)
        {}

        CONSTEXPR11_TN static_string(const char* p, std::size_t N) NOEXCEPT_TN
                : p_(p)
                , sz_(N)
        {}

        CONSTEXPR11_TN const char* data() const NOEXCEPT_TN {return p_;}
        CONSTEXPR11_TN std::size_t size() const NOEXCEPT_TN {return sz_;}

        CONSTEXPR11_TN const_iterator begin() const NOEXCEPT_TN {return p_;}
        CONSTEXPR11_TN const_iterator end()   const NOEXCEPT_TN {return p_ + sz_;}

        CONSTEXPR11_TN char operator[](std::size_t n) const
        {
            return n < sz_ ? p_[n] : throw std::out_of_range("static_string");
        }
    };
    inline
    std::ostream&
    operator<<(std::ostream& os, static_string const& s)
    {
        return os.write(s.data(), s.size());
    }


    /**
     *
     * @tparam T
     * @return
     */
    template <class T>
    CONSTEXPR14_TN
    static_string
    type_name();

    template <typename R, typename ... Types>
    constexpr std::integral_constant<unsigned, sizeof ...(Types)> countArgs( R(*f)(Types ...))
    {
        return std::integral_constant<unsigned, sizeof ...(Types)>{};
    }

    /**
     * utility function for erasing element in given vector
     * @tparam V value type of the given vector
     * @param vector where to erase the value from
     * @param valueToErase
     */
    template <class V>
    void erase(std::vector<V>& vector, V valueToErase) {
        vector.erase( std::remove( vector.begin(), vector.end(), valueToErase ), vector.end() );
    }

    /**
     * Utility function for erasing all elements matching the given predicate function
     * in given vector
     * @tparam V value type of the given vector
     * @tparam Predicate
     * @param v where to erase the values from
     * @param predicate function accepting V and returning bool
     */
    template <class V, typename Predicate>
    void erase(std::vector<V>& v, Predicate predicate) {
        v.erase( std::remove_if(v.begin(), v.end(), predicate), v.end() );
    }

    class SubscriptionHandle{
        using Sub = std::function<void()>;
        Sub removeSub;
        bool doOnce = true;
    public:
        SubscriptionHandle(Sub&& lambda) : removeSub{std::forward<Sub>(lambda)} {}
        void unsubscribe(){
            std::cout << "unsub called \n";
            if(doOnce){
                doOnce = false;
                removeSub();
                std::cout << "after unsub called \n";
            }
        }
    };


    template <class V >
    class Var{
        V _v;
        std::list<std::function<void(V)>> _listeners;
        std::function<void()> _notifyAll;
    public:
        Var(V initialValue);
        V get() const noexcept;

        template <class Func, class Callee>
        std::unique_ptr<SubscriptionHandle> onUpdate(Callee&& callee, Func&& callback);

        std::unique_ptr<SubscriptionHandle> onUpdate(const std::function<void(V)>& callback);

        long listenerSize();

        void set(const V& newValue) noexcept;

    };

#include "Var.cpp"
#include "TypeUtil.cpp"
}
#endif //EPIPHANY_U_H
