#include "agt_asensing_driver/asensing_parser.hpp"
#include <gtest/gtest.h>

using agt_asensing_driver::ASENSINGParser;
TEST(ASENSINGParser, ParsesFragmentedValidMainFrame)
{
  std::vector<uint8_t> frame(58, 0);
  frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[21] = 0x78; frame[22] = 0x56; frame[23] = 0x34; frame[24] = 0x12;
  frame[25] = 0xf0; frame[26] = 0xde; frame[27] = 0xbc; frame[28] = 0x9a;
  frame[39] = 3;
  for (std::size_t i = 0; i < 57; ++i) frame[57] ^= frame[i];
  ASENSINGParser parser;
  EXPECT_TRUE(parser.feed(frame.data(), 20).empty());
  const auto result = parser.feed(frame.data() + 20, frame.size() - 20);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].latitude, 305419896 * 1e-7, 1e-10);
  EXPECT_NEAR(result[0].longitude, -1698898192 * 1e-7, 1e-10);
  EXPECT_EQ(result[0].ins_status, 3);
}

TEST(ASENSINGParser, RejectsBadChecksum)
{
  std::vector<uint8_t> frame(58, 0); frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[57] = 1;
  EXPECT_TRUE(ASENSINGParser().feed(frame).empty());
}
