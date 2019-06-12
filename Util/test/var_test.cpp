//
// Created by sigi on 31.05.19.
//

#include <list>
#include "U.h"
#include "gtest/gtest.h"

void show(int i) {
    std::cout<< "\n";
    std::cout<<"value update: " << i << "\n";
}
struct Stud{
    U::Var<std::string> name;
    U::SubscriptionHandle handle = name.onUpdate(&Stud::print, this);
    Stud() : name("peter"){}

    void hello(){}
    void show(int i) {
        std::cout<< "\n";
        std::cout<<"value update: " << i << "\n";
    }
    void print(std::string s){
        std::cout<< "\n";
        std::cout<<"string update: " << s << "\n";
        handle.unsubscribe();
    }
};

template <class T>
bool eq(T* a, T* b) {
    return a == b;
};

template <class T>
bool eq(T& a, T& b) {
    return std::addressof(a) == std::addressof(b);
}   ;
bool eq(const std::function<void()>& a, const std::function<void()>& b) {

    return b.target<void()>() == a.target<void()>();
};
TEST(var_test, some_test){
    std::vector<Stud*> studs;
    Stud s1;
    Stud s2;
    auto ref1 = std::ref(s1);
    auto ref2 = std::ref(s2);
    std::cout<< "\n";
    std::cout<< "is equal : " << eq(std::bind(&Stud::hello, &s1), std::bind(&Stud::hello, &ref2.get())) << "\n";
}

TEST(var_test, check_method_equality){
    std::list<std::reference_wrapper<const std::function<void(int)>>> listeners;
    const std::function<void(int)> l = [](int i){std::cout<<"\n shit called \n";};
    const std::function<void(int)> g = [](int i){std::cout<<"\n shit load called \n";};
    listeners.push_front(std::cref(l));
    auto it = listeners.begin();

    listeners.push_front(std::cref(g));
    auto it2 = listeners.begin();
    it.operator->()->get()(2);
    it2.operator->()->get()(2);
    listeners.erase(it2, std::next(it2,1));
    std::cout<<"size of list: " << listeners.size() << "\n";
    it.operator->()->get()(2);
    it2.operator->()->get()(2);

}

TEST(var_test, add_listener){
    U::Var<int> v(42);
    Stud s;
    auto c = [](int v){std::cout << "alter changed : " << v << "\n";};
    auto subHandle = v.onUpdate(c);
    v.set(1337);
    //subHandle.unsubscribe();
    v.set(42);
    s.name.set("hans");
    s.name.set("maulwurf");

}

TEST(var_test, set_get_test){
    U::Var<int> v(42);
    v.set(1337);
    EXPECT_EQ(1337, v.get());
}
