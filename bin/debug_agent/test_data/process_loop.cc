// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <unistd.h>

int main() {
  // Run for 20 seconds and then end.
  for (size_t i = 0; i < 20; i++) {
    sleep(1);
  }
}
