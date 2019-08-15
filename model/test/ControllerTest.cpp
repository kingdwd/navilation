//
// Created by sigi on 12.06.19.
//

#include "data.h"
#include <gtest/gtest.h>
#include "../src/StateBlock.cpp"
#include <math.h>


TEST(random_test, tester){
struct T{

    std::function<void()> t;
    T(std::function<void()>&& t) : t{t}{};
};
auto t = std::make_unique<T>([this]{std::cout<<"HELLO WOHOHO\n";});
t->t();
}
