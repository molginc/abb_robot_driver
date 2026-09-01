#ifndef ABB_RWS_STATE_PUBLISHER_POLL_GUARD_H
#define ABB_RWS_STATE_PUBLISHER_POLL_GUARD_H

#include <Poco/Exception.h>

#include <functional>
#include <string>

namespace abb
{
namespace robot
{
/**
 * \brief Exception barrier for the cyclic RWS poll and publish steps.
 *
 * RWS communication surfaces a mix of exception types: Poco exceptions for
 * transport and XML-parse failures (which do NOT derive from
 * std::runtime_error), plus std::runtime_error and std::logic_error from
 * abb_egm_rws_managers. None of them may escape a ros::Timer callback — an
 * escaped exception unwinds ros::spin() and terminates the node, which tears
 * down the whole launch when the node is marked required.
 */
class PollGuard
{
public:
  /**
   * \brief Invokes the callable, converting any thrown exception into a false return.
   *
   * \param callable the work to guard.
   *
   * \return bool true if the callable completed without throwing.
   */
  bool run(const std::function<void()>& callable)
  {
    try
    {
      callable();
      consecutive_failures_ = 0;
      last_error_.clear();
      return true;
    }
    catch (const Poco::Exception& exception)
    {
      // Poco's what() is only the exception class name; displayText() carries
      // the actual failure message (e.g. the XML parse error).
      return recordFailure(exception.displayText());
    }
    catch (const std::exception& exception)
    {
      return recordFailure(exception.what());
    }
    catch (...)
    {
      return recordFailure("unknown exception");
    }
  }

  /**
   * \brief Number of run() calls that have failed since the last success.
   */
  unsigned int consecutiveFailures() const
  {
    return consecutive_failures_;
  }

  /**
   * \brief Message of the most recent failure (empty after a success).
   */
  const std::string& lastError() const
  {
    return last_error_;
  }

private:
  bool recordFailure(const std::string& message)
  {
    ++consecutive_failures_;
    last_error_ = message;
    return false;
  }

  unsigned int consecutive_failures_{ 0 };
  std::string last_error_{};
};

}  // namespace robot
}  // namespace abb

#endif
