//
// Created by sigi on 12.06.19.
//

#include "UMath.h"
#include <gtest/gtest.h>

using namespace U::Math;
TEST(sin, check_base_values_are_correct){
    EXPECT_EQ(0.0, sind(0.0));
    EXPECT_EQ(0, sind(0));
    EXPECT_EQ(0, sind(180.0));
    EXPECT_EQ(0, sind(-180.0));
    EXPECT_NE(0, sind(-180.1));

    EXPECT_EQ(1, sind(90.0));
    EXPECT_EQ(1, sind(90));
    EXPECT_EQ(1, sind(-270));
    EXPECT_EQ(-1, sind(-90));
    EXPECT_EQ(-1, sind(270.0));

    EXPECT_NEAR(0.5, sind(30.0), 0.000000001);
    EXPECT_NEAR(-0.5, sind(-30.0), 0.000000001);
}

TEST(cos, check_base_values_are_correct){
    EXPECT_EQ(1.0, cosd(0.0));
    EXPECT_EQ(1, cosd(0));
    EXPECT_EQ(-1, cosd(180.0));
    EXPECT_EQ(-1, cosd(-180.0));
    EXPECT_NE(-1, cosd(-180.1));

    EXPECT_EQ(0, cosd(90.0));
    EXPECT_EQ(0, cosd(90));
    EXPECT_EQ(0, cosd(-270));
    EXPECT_EQ(0, cosd(-90));
    EXPECT_EQ(0, cosd(270.0));

    EXPECT_NEAR(0.5, cosd(60.0), 0.000000001);
    EXPECT_NEAR(0.5, cosd(-60.0), 0.000000001);
    EXPECT_NEAR(-0.5, cosd(120.0), 0.000000001);
    EXPECT_NEAR(-0.5, cosd(-120.0), 0.000000001);
}

TEST(cos, failure_test){
    EXPECT_NEAR(0.5, cosd(-60.0), 0.000001);
}