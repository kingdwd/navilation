//
// Created by sigi on 24.08.19.
//

#include <gtest/gtest.h>
#include "limit_block.hpp"

TEST(limit_block_test, test_limit){
    epi::limit_block limitBlock(4);
    
    ASSERT_EQ(4, limitBlock.apply(34));
}
