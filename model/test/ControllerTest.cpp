//
// Created by sigi on 12.06.19.
//

#include "data.h"
#include <gtest/gtest.h>
#include <math.h>


TEST(simple_controller, test_pose) {
    epi::SimpleController controller;
    epi::Pose expectedPose{0, 1, 90};
    std::cout<< " \n sin of 90: " << std::sin(1.57) << "\n";
    EXPECT_EQ(expectedPose, controller.convert(1, 0, epi::Pose{0,0, 90}));
}
