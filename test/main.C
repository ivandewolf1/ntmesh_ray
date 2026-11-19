#include <iostream>
#include "tests.h"

int g_fails = 0;

int main()
{
  rayTest();

  if (g_fails) {
    std::cerr << g_fails << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "all assertions passed\n";
  return 0;
}
