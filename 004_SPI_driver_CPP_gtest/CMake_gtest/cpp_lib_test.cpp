/*File nay de test unit test cho main.cpp*/

#include "cpplib.h"
#include "gtest/gtest.h"

TEST(cpp_lib_test, ReturnHelloWorld) {
    cpp_lib cpplib;
    string actual = cpplib.print_hello_world();
    string expected = "Hello World";
    EXPECT_EQ(expected, actual);
}

TEST(cpp_lib_test, findMax) {
    cpp_lib cpplib;
    vector<int> input = {5, 3, 1, 50, 77};
    int actual_max = cpplib.find_max(input);
    EXPECT_EQ(actual_max, 77);
    EXPECT_EQ(cpplib.find_max({3}), 3); // test 1 element
    EXPECT_EQ(cpplib.find_max({}), -1) << "list empty return fail"; // test empty
}