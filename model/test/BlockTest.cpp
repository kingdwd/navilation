//
// Created by sigi on 24.08.19.
//

#include <gtest/gtest.h>
#include "LimitBlock.hpp"

TEST(limit_block_test, test_limit){
    epi::LimitBlock limitBlock(4);
    
    ASSERT_EQ(4, limitBlock.apply(34));
}
