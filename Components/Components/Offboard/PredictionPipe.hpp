#pragma once
#include <mutex>
#include <deque>
#include <Eigen/Dense>
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Time/ManualTimer.hpp"


//TODO FIXME: this needs to be thread-safe, mutex

namespace Offboard {

template<class T_msg>
class PredictionPipe {
  static constexpr double SMALL_TIME = 1e-6;  // [s]
public:
  PredictionPipe(BaseTimer* const timer, double delayTime)
      : _timer(timer),
        _delayTime(delayTime) {
    //Do nothing
  }

  void AddMessage(T_msg msg) {
    TimedMessage tmsg;
    tmsg.timeActive = _timer.GetSeconds<double>() + _delayTime;  //message is sent now, will be active after _delayTime
    tmsg.msg = msg;
    _messages.push_back(tmsg);
  }

  bool GetActiveMessage(double t, T_msg &out, double &timeRemaining) const {
    if (!_messages.size()) {
      return false;
    }
    //beginning at the end, find the newest message that's valid now.
    // Returns false if no valid message exists.
    auto msg = _messages.cend();
    double tLastMsg = 1e10;
    for (;;) {
      msg--;
      if ((t + SMALL_TIME) >= msg->timeActive) {
        out = msg->msg;
        timeRemaining = tLastMsg - msg->timeActive;
        return true;
      }
      tLastMsg = msg->timeActive;
      if (msg == _messages.cbegin()) {
        break;
      }
    }
    return false;
  }

  void ClearExpiredMessages(double currentTime) {
    int N = _messages.size();
    //this is inelegant
    for (int i = 0; i < N; i++) {
      if (_messages.size() < 2) {
        //last message always stays active.
        return;
      }

      if (_messages[1].timeActive <= currentTime) {
        _messages.pop_front();
      }
    };
  }

 private:
  struct TimedMessage {
    double timeActive;  //[s]
    T_msg msg;
  };

  Timer _timer;
  double _delayTime;
  std::deque<TimedMessage> _messages;

};
    
}  // namespace Offboard