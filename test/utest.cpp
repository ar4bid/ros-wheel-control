#include "sabertoothWrite.cpp"
#include <gtest/gtest.h>

TEST(readFromController, invalidAddress){
	EXPECT_EQ(5,5);
}

TEST(readFromController, invalidCommand){
	EXPECT_EQ(5,4);
}
/*
TEST(readFromController, invalidData){

}
*/
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
//	ros::init(argc, argv, "tester");
//	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}
