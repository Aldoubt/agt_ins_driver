#include <gtest/gtest.h>
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>
#include "agt_asensing_driver/serial_port.hpp"

TEST(SerialPort, OpensFieldReceiverAt460800)
{
  int master = -1, slave = -1;
  char name[256]{};
  ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
  agt_asensing_driver::SerialPort serial;
  const bool opened = serial.open(name, 460800);
  EXPECT_TRUE(opened);
  if (opened) {
    termios settings{};
    ASSERT_EQ(tcgetattr(slave, &settings), 0);
    EXPECT_EQ(cfgetispeed(&settings), B460800);
    EXPECT_EQ(cfgetospeed(&settings), B460800);
  }
  serial.close();
  close(slave);
  close(master);
}
