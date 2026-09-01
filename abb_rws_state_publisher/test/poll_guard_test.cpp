#include <abb_rws_state_publisher/poll_guard.h>

#include <gtest/gtest.h>

#include <Poco/SAX/SAXException.h>

#include <stdexcept>

namespace abb
{
namespace robot
{
TEST(PollGuardTest, RunsTheCallableAndReturnsTrueOnSuccess)
{
  PollGuard guard{};
  bool ran{ false };

  EXPECT_TRUE(guard.run([&ran] { ran = true; }));
  EXPECT_TRUE(ran);
  EXPECT_EQ(guard.consecutiveFailures(), 0u);
}

TEST(PollGuardTest, SwallowsPocoSaxParseException)
{
  // Regression: 2026-08-31 station shutdown. Poco exceptions do not derive
  // from std::runtime_error, so the callback's original catch let a
  // SAXParseException unwind ros::spin() and kill the (required) node.
  PollGuard guard{};

  EXPECT_FALSE(guard.run([] { throw Poco::XML::SAXParseException{ "bad controller XML", "", "", 1, 0 }; }));
  EXPECT_EQ(guard.consecutiveFailures(), 1u);
  EXPECT_NE(guard.lastError().find("bad controller XML"), std::string::npos);
}

TEST(PollGuardTest, SwallowsLogicErrorAndNonStdExceptions)
{
  // collectAndUpdateRuntimeData also throws std::logic_error ("System name
  // mismatch"), and anything else unexpected must not escape either.
  PollGuard guard{};

  EXPECT_FALSE(guard.run([] { throw std::logic_error{ "System name mismatch" }; }));
  EXPECT_NE(guard.lastError().find("System name mismatch"), std::string::npos);

  EXPECT_FALSE(guard.run([] { throw 42; }));
  EXPECT_FALSE(guard.lastError().empty());
}

TEST(PollGuardTest, CountsConsecutiveFailuresAndResetsOnSuccess)
{
  PollGuard guard{};

  guard.run([] { throw std::runtime_error{ "poll failed" }; });
  guard.run([] { throw std::runtime_error{ "poll failed" }; });
  EXPECT_EQ(guard.consecutiveFailures(), 2u);

  EXPECT_TRUE(guard.run([] {}));
  EXPECT_EQ(guard.consecutiveFailures(), 0u);
  EXPECT_TRUE(guard.lastError().empty());
}

}  // namespace robot
}  // namespace abb
