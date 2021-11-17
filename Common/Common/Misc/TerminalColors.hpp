#pragma once

#include <stdio.h>

namespace TerminalColors {

enum Color {
  RED,
  GREEN,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN,
  WHITE,
  RESET,
};

inline void SetTerminalColor(Color c) {
  switch (c) {
    case RED:
      printf("\x1B[31m");
      return;
    case GREEN:
      printf("\x1B[32m");
      return;
    case YELLOW:
      printf("\x1B[33m");
      return;
    case BLUE:
      printf("\x1B[34m");
      return;
    case MAGENTA:
      printf("\x1B[35m");
      return;
    case CYAN:
      printf("\x1B[36m");
      return;
    case WHITE:
      printf("\x1B[37m");
      return;
    default:
      printf("\x1B[0m");
      return;
  }
}

inline void ResetTerminalColor() {
  SetTerminalColor(RESET);
}
}
