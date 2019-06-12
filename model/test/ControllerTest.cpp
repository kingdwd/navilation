//
// Created by sigi on 12.06.19.
//

#include "data.h"
#include <gtest/gtest.h>
#include <math.h>

epi::SimpleController controller;

TEST(simple_controller, move_forward_from_90_deg) {
    epi::Pose expectedPose{0, 1, 90};
    EXPECT_EQ(expectedPose, controller.convert(1, 0, epi::Pose{0,0, 90}));
}

TEST(simple_controller, move_backward_from_90_deg) {
    epi::Pose expectedPose{0, 0, 90};
    EXPECT_EQ(expectedPose, controller.convert(-1, 0, epi::Pose{0,1, 90}));
}

TEST(simple_controller, move_backward_from_270_deg) {
    epi::Pose expectedPose{0, 1, 270};
    EXPECT_EQ(expectedPose, controller.convert(-1, 0, epi::Pose{0,0, 270}));
}

TEST(simple_controller, move_forward_from_0_deg) {
    epi::Pose expectedPose{1, 0, 0};
    EXPECT_EQ(expectedPose, controller.convert(1, 0, epi::Pose{0,0,0}));
}
TEST(simple_controller, move_backward_from_0_deg) {
    epi::Pose expectedPose{0, 0, 0};
    EXPECT_EQ(expectedPose, controller.convert(-1, 0, epi::Pose{1,0,0}));
}
