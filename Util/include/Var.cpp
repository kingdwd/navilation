//
// Created by sigi on 31.05.19.
//
#include "U.h"

//template<class T>
//void U::SubscriptionHandle<T>::unsubscribe() {
//    lambda();
//}

template <class V>
U::Var<V>::Var(const V initialValue) :_v(initialValue) {}

template <class V>
V U::Var<V>::get() const noexcept { return _v; }

template <class V>
void U::Var<V>::set(const V newValue) noexcept {
    _v = newValue;
    auto l = _listeners;
    auto iter = l.begin();
    while(iter != l.end()){
        (*iter)(newValue);
        iter++;
    }
}


template<class V>
template<class Func, class Callee>
U::SubscriptionHandle U::Var<V>::onUpdate(Func &&callback, Callee &&callee) {
    using namespace std::placeholders;
    std::function<void(V)> cb = std::bind(std::forward<Func>(callback)
            ,std::forward<Callee>(callee)
            , std::forward<decltype(_1)>(_1));
    _listeners.push_front(cb);
    return SubscriptionHandle([this, it = std::move(_listeners.cbegin())](){ _listeners.erase(it); });
}

template<class V>
U::SubscriptionHandle U::Var<V>::onUpdate(const std::function<void(V)> &callback) {
    _listeners.push_front(callback);
    return SubscriptionHandle([this, it = std::move(_listeners.cbegin())](){ _listeners.erase(it); });
}

