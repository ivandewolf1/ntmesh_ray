#pragma once

#include <iostream>
#include "ntmesh.h"

extern int g_fails;

inline void expect_true(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_fails;
  }
}

template <typename T>
inline void expect_vec(const ntv::nvec3<T>& got, const ntv::nvec3<T>& exp, T eps, const char* what) {
  expect_true(got.almostEqual(exp, eps), what);
}

void rayTest();
