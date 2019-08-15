//
// Created by sigi on 31.05.19.
//
#include "U.h"

//template<class T>
//void U::SubscriptionHandle<T>::unsubscribe() {
//    lambda();
//}

template <class V>
U::Var<V>::Var(const V initialValue) : _v(initialValue) {
    _notifyAll = [this] {
        for (auto listener = _listeners.begin(); listener != _listeners.end(); listener++) {
            (*listener)(_v);
        }
    };
}

template <class V>
V U::Var<V>::get() const noexcept { return _v; }

template <class V>
void U::Var<V>::set(const V& newValue) noexcept {
    if(_v == newValue) return;
    _v = newValue;
    if(!_listeners.empty()){
        std::async(std::launch::async, _notifyAll);
    }
}


template<class V>
template<class Func, class Callee>
std::unique_ptr<U::SubscriptionHandle> U::Var<V>::onUpdate(Callee &&callee, Func&& callback) {
    using namespace std::placeholders;
    const std::function<void(V)> cb = std::bind(std::forward<Func>(callback)
            ,std::forward<Callee>(callee)
            , std::forward<decltype(_1)>(_1));
    return onUpdate(cb);
}

template<class V>
std::unique_ptr<U::SubscriptionHandle> U::Var<V>::onUpdate(const std::function<void(V)> &callback) {
    _listeners.push_front(callback);
    return std::make_unique<SubscriptionHandle>([this, it = std::move(_listeners.cbegin())](){
        std::cout<<"before unsub: " << _listeners.size() << "\n";
        _listeners.erase(it);
        std::cout<<"after unsub: " << _listeners.size() << "\n";
    });
}

template<class V>
long U::Var<V>::listenerSize() {
    return _listeners.size();
}

