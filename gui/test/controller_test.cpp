//
// Created by sigi on 18.08.19.
//
#include <gtest/gtest.h>
#include <math.h>

TEST(random_test, tester){
struct T{

    std::function<void()> t;
    T(std::function<void()>&& t) : t{t}{};
};
auto t = std::make_unique<T>([this]{std::cout<<"HELLO WOHOHO\n";});
t->t();
}
