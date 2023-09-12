#include <gmock/gmock.h>
#include "MyGlobalParameters.h"

using ::testing::_;
using ::testing::Mock;
using ::testing::Exactly;

TEST(MyGlobalParametersTester, SingeltonTest) 
{
  MyGlobalParameters* params1 = &MyGlobalParameters::getInstance();
  MyGlobalParameters* params2 = &MyGlobalParameters::getInstance();
  EXPECT_TRUE( params1 == params2 );
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fkie_GlobalParameter_test");
  return RUN_ALL_TESTS();
}
