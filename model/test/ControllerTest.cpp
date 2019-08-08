//
// Created by sigi on 12.06.19.
//

#include "data.h"
#include <gtest/gtest.h>
#include "../src/StateBlock.cpp"
#include <math.h>

epi::SimpleController controller;

TEST(simple_controller, move_forward_from_90_deg) {
    epi::State expectedState{0, 1, 90};
    EXPECT_EQ(expectedState, controller.convert(1, 0, epi::State{0,0, 90}));
}

TEST(simple_controller, move_backward_from_90_deg) {
    epi::State expectedState{0, 0, 90};
    EXPECT_EQ(expectedState, controller.convert(-1, 0, epi::State{0,1, 90}));
}

TEST(simple_controller, move_backward_from_270_deg) {
    epi::State expectedState{0, 1, 270};
    EXPECT_EQ(expectedState, controller.convert(-1, 0, epi::State{0,0, 270}));
}

TEST(simple_controller, move_forward_from_0_deg) {
    epi::State expectedState{1, 0, 0};
    EXPECT_EQ(expectedState, controller.convert(1, 0, epi::State{0,0,0}));
}
TEST(simple_controller, move_backward_from_0_deg) {
    epi::State expectedState{0, 0, 0, 0,0,0};
    EXPECT_EQ(expectedState, controller.convert(-1, 0, epi::State{1,0,0,0,0,0}));
}

TEST(integrator_block, works_as_expected){
    epi::State s{1,0,0, 0,0,0};
    epi::State expectedState{2, 0, 0, 0,0,0};
    epi::IntegratorBlock<epi::State> i{s};
    EXPECT_EQ(expectedState, i.apply(s));
}

TEST(deadband_test, deadband_blocks){
    epi::Deadband deadband(5);
    EXPECT_EQ(0, deadband.apply(3));
}

TEST(deadband_test, deadband_passes){
    epi::Deadband deadband(5);
    EXPECT_EQ(1, deadband.apply(6));
    EXPECT_EQ(-1, deadband.apply(-6));
}

TEST(limit_block_test, returns_value_when_within_the_limit) {
    epi::LimitBlock<int> limit{5};
    EXPECT_EQ(3, limit.apply(3));
    EXPECT_EQ(0, limit.apply(0));
    EXPECT_EQ(-3, limit.apply(-3));
}

TEST(limit_block_test, some_cv_tests){
    cv::Vec6d vec;
    for(int i=0; i<6; i++){
        vec[i] = i;
    }

    cv::Vec2d point{5, 10};
    cv::Mat m = cv::Mat::zeros(6, 2, CV_64F);

    cv::Vec6d c1 = vec.all(1);
    std::vector<cv::Vec2d> points;
    for(auto& c : vec.val){
        points.push_back(c*point);
    }
    for(auto& i : points){
        std::cout<< "vec * scalar" << i << "\n";
    }
    //std::cout<< "vec * mat" << m.mul(vec) << "\n";
}
TEST(limit_block_test, returns_limit_when_value_exceeds_limit) {
    epi::LimitBlock<int> limit{5};
    EXPECT_EQ(5, limit.apply(6));
    EXPECT_EQ(-5, limit.apply(-6));
}
