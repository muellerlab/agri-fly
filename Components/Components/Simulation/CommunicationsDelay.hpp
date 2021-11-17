#pragma once

#include <queue>

#include "Common/Time/Timer.hpp"

namespace Simulation {

template<class T_msg>
class CommunicationsDelay {
 public:
  CommunicationsDelay(BaseTimer* const timer, double delayTime)
      : _timer(timer),
        _delayTime_us(uint64_t(delayTime * 1e6)) {
    //Do nothing
  }

  void AddMessage(T_msg msg) {
    TimedMessage tmsg;
    tmsg.timeToSend = _timer.GetMicroSeconds() + _delayTime_us;
    tmsg.msg = msg;
    _messages.push(tmsg);
  }

  bool HaveNewMessage(void) {
    if (!_messages.size()) {
      //nothing in queue
      return false;
    }
    uint64_t tNextMsg = _messages.front().timeToSend;
    uint64_t tCurr = _timer.GetMicroSeconds();
    return tCurr >= tNextMsg;
  }

  T_msg GetMessage(void) {
    T_msg out = _messages.front().msg;
    _messages.pop();
    return out;
  }

 private:
  struct TimedMessage {
    uint64_t timeToSend;
    T_msg msg;
  };

  Timer _timer;
  uint64_t _delayTime_us;
  std::queue<TimedMessage> _messages;

};

}  // namespace Simulation

