#pragma once

namespace Onboard {

enum PanicReason {
  PANIC_NO_PANIC = 0,
  PANIC_ONBOARD_ESTIMATE_CRAZY = 1,
  PANIC_UWB_TIMEOUT = 2,
  PANIC_UPSIDE_DOWN = 3,
  PANIC_RADIO_CMD_TIMEOUT = 4,
  PANIC_LOW_BATTERY = 5,
  PANIC_KILLED_INTERNALLY = 6,
  PANIC_KILLED_EXTERNALLY = 7,
};

static inline char const* GetPanicReasonString(PanicReason p) {
  switch (p) {
    case PANIC_NO_PANIC:
      return "NO_PANIC";
    case PANIC_ONBOARD_ESTIMATE_CRAZY:
      return "ONBOARD_ESTIMATE_CRAZY";
    case PANIC_UWB_TIMEOUT:
      return "UWB_TIMEOUT";
    case PANIC_UPSIDE_DOWN:
      return "UPSIDE_DOWN";
    case PANIC_RADIO_CMD_TIMEOUT:
      return "RADIO_CMD_TIMEOUT";
    case PANIC_LOW_BATTERY:
      return "LOW_BATTERY";
    case PANIC_KILLED_INTERNALLY:
      return "KILLED_INTERNALLY";
    case PANIC_KILLED_EXTERNALLY:
      return "KILLED_EXTERNALLY";
    default:
      return "UNKNOWN_PANIC!";
  }
}

}
