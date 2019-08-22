//
// Created by sigi on 11.12.18.
//

#include "U.h"
#include "gtest/gtest.h"


bool isEven(int i) {return i%2 == 0;}

TEST(basic_check, erase_deletes_element_in_vector) {
    std::vector<int> i{1,2,3,4,5,6};
    U::erase(i, 4);
    EXPECT_EQ(i.size(), 5);
}

TEST(basic_check, erase_doesnt_delete_non_existing_element_in_vector) {
    std::vector<int> i{1,2,3,4,5,6};
    U::erase(i, 7);
    EXPECT_EQ(i.size(), 6);
}

TEST(basic_check, erase_deletes_all_elements_based_on_predicate) {
    std::vector<int> i{1,2,3,4,5,6};
    U::erase(i, isEven);
    EXPECT_EQ(i.size(), 3);
}

TEST(basic_check, for_each_iterates){
    std::vector<int> i{1,2,3,4,5,6};
    U::foreach(i, []{std})

}
