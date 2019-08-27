// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <zircon/system/public/zircon/syscalls/port.h>

// TODO(fidlcat): Look into this.  Removing the hack that led to this (in
// debug_ipc/helper/message_loop.h) seems to work, except it breaks SDK builds
// on CQ in a way I can't repro locally.
#undef __TA_REQUIRES

#include "gtest/gtest.h"
#include "interception_workflow_test.h"

namespace fidlcat {

std::string ClockExpected(time_t time, const char* format);

constexpr uint64_t kKey = 1234;
constexpr uint64_t kSignalCount = 2;
constexpr uint64_t kSource = 0xab1234;

// zx_port_create tests.

std::unique_ptr<SystemCallTest> ZxPortCreate(int64_t status, std::string_view status_name,
                                             uint32_t options, zx_handle_t* out) {
  auto value = std::make_unique<SystemCallTest>("zx_port_create", status, status_name);
  value->AddInput(options);
  value->AddInput(reinterpret_cast<uint64_t>(out));
  return value;
}

#define PORT_CREATE_DISPLAY_TEST_CONTENT(status, expected) \
  zx_handle_t handle = kHandle;                            \
  PerformDisplayTest("zx_port_create@plt", ZxPortCreate(status, #status, 0, &handle), expected);

#define PORT_CREATE_DISPLAY_TEST(name, status, expected) \
  TEST_F(InterceptionWorkflowTestX64, name) {            \
    PORT_CREATE_DISPLAY_TEST_CONTENT(status, expected);  \
  }                                                      \
  TEST_F(InterceptionWorkflowTestArm, name) { PORT_CREATE_DISPLAY_TEST_CONTENT(status, expected); }

PORT_CREATE_DISPLAY_TEST(
    ZxPortCreate, ZX_OK,
    "\ntest_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
    "zx_port_create(options:\x1B[32muint32\x1B[0m: \x1B[34m0\x1B[0m)\n"
    "  -> \x1B[32mZX_OK\x1B[0m (out:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m)\n");

// zx_port_queue tests.

std::unique_ptr<SystemCallTest> ZxPortQueue(int64_t status, std::string_view status_name,
                                            zx_handle_t handle, zx_port_packet_t* packet) {
  auto value = std::make_unique<SystemCallTest>("zx_port_queue", status, status_name);
  value->AddInput(handle);
  value->AddInput(reinterpret_cast<uint64_t>(packet));
  return value;
}

#define PORT_QUEUE_DISPLAY_TEST_CONTENT(status, handle, init_packet, expected) \
  zx_port_packet_t packet;                                                     \
  init_packet(&packet);                                                        \
  PerformDisplayTest("zx_port_queue@plt", ZxPortQueue(status, #status, handle, &packet), expected);

#define PORT_QUEUE_DISPLAY_TEST(name, status, handle, init_packet, expected) \
  TEST_F(InterceptionWorkflowTestX64, name) {                                \
    PORT_QUEUE_DISPLAY_TEST_CONTENT(status, handle, init_packet, expected);  \
  }                                                                          \
  TEST_F(InterceptionWorkflowTestArm, name) {                                \
    PORT_QUEUE_DISPLAY_TEST_CONTENT(status, handle, init_packet, expected);  \
  }

void InitUser(zx_port_packet_t* packet) {
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_USER;
  packet->status = ZX_OK;
  packet->user.u64[0] = 0x123456789abcdef0UL;
  packet->user.u64[1] = 0x3456789abcdef012UL;
  packet->user.u64[2] = 0x56789abcdef01234UL;
  packet->user.u64[3] = 0x789abcdef0123456UL;
}

PORT_QUEUE_DISPLAY_TEST(
    ZxPortQueueUser, ZX_OK, kHandle, InitUser,
    "\n"
    "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
    "zx_port_queue("
    "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m)\n"
    "    packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
    "      key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
    "      type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_USER\x1B[0m\n"
    "      status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
    "      user:\x1B[32mzx_packet_user_t\x1B[0m: {\n"
    "        u64:\x1B[32muint64[]\x1B[0m: "
    "\x1B[34m123456789abcdef0\x1B[0m, "
    "\x1B[34m3456789abcdef012\x1B[0m, "
    "\x1B[34m56789abcdef01234\x1B[0m, "
    "\x1B[34m789abcdef0123456\x1B[0m\n"
    "        u32:\x1B[32muint32[]\x1B[0m: "
    "\x1B[34m9abcdef0\x1B[0m, \x1B[34m12345678\x1B[0m, "
    "\x1B[34mbcdef012\x1B[0m, \x1B[34m3456789a\x1B[0m, "
    "\x1B[34mdef01234\x1B[0m, \x1B[34m56789abc\x1B[0m, "
    "\x1B[34mf0123456\x1B[0m, \x1B[34m789abcde\x1B[0m\n"
    "        u16:\x1B[32muint16[]\x1B[0m: "
    "\x1B[34mdef0\x1B[0m, \x1B[34m9abc\x1B[0m, \x1B[34m5678\x1B[0m, \x1B[34m1234\x1B[0m, "
    "\x1B[34mf012\x1B[0m, \x1B[34mbcde\x1B[0m, \x1B[34m789a\x1B[0m, \x1B[34m3456\x1B[0m, "
    "\x1B[34m1234\x1B[0m, \x1B[34mdef0\x1B[0m, \x1B[34m9abc\x1B[0m, \x1B[34m5678\x1B[0m, "
    "\x1B[34m3456\x1B[0m, \x1B[34mf012\x1B[0m, \x1B[34mbcde\x1B[0m, \x1B[34m789a\x1B[0m\n"
    "        u8:\x1B[32muint8[]\x1B[0m: "
    "\x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m,"
    " \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, "
    "\x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m,"
    " \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, "
    "\x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m,"
    " \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, "
    "\x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m,"
    " \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m\n"
    "      }\n"
    "    }\n"
    "  -> \x1B[32mZX_OK\x1B[0m\n");

// zx_port_wait tests.

std::unique_ptr<SystemCallTest> ZxPortWait(int64_t status, std::string_view status_name,
                                           zx_handle_t handle, zx_time_t deadline,
                                           zx_port_packet_t* packet) {
  auto value = std::make_unique<SystemCallTest>("zx_port_wait", status, status_name);
  value->AddInput(handle);
  value->AddInput(deadline);
  value->AddInput(reinterpret_cast<uint64_t>(packet));
  return value;
}

#define PORT_WAIT_DISPLAY_TEST_CONTENT(status, handle, deadline, init_packet, expected)          \
  zx_port_packet_t packet;                                                                       \
  init_packet(&packet);                                                                          \
  PerformDisplayTest("zx_port_wait@plt", ZxPortWait(status, #status, handle, deadline, &packet), \
                     expected);

#define PORT_WAIT_DISPLAY_TEST(name, status, handle, deadline, init_packet, expected) \
  TEST_F(InterceptionWorkflowTestX64, name) {                                         \
    PORT_WAIT_DISPLAY_TEST_CONTENT(status, handle, deadline, init_packet, expected);  \
  }                                                                                   \
  TEST_F(InterceptionWorkflowTestArm, name) {                                         \
    PORT_WAIT_DISPLAY_TEST_CONTENT(status, handle, deadline, init_packet, expected);  \
  }

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitUser, ZX_OK, kHandle, ZX_TIME_INFINITE, InitUser,
    "\n"
    "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
    "zx_port_wait("
    "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
    "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
    "  -> \x1B[32mZX_OK\x1B[0m\n"
    "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
    "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
    "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_USER\x1B[0m\n"
    "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
    "        user:\x1B[32mzx_packet_user_t\x1B[0m: {\n"
    "          u64:\x1B[32muint64[]\x1B[0m: "
    "\x1B[34m123456789abcdef0\x1B[0m, "
    "\x1B[34m3456789abcdef012\x1B[0m, "
    "\x1B[34m56789abcdef01234\x1B[0m, "
    "\x1B[34m789abcdef0123456\x1B[0m\n"
    "          u32:\x1B[32muint32[]\x1B[0m: "
    "\x1B[34m9abcdef0\x1B[0m, \x1B[34m12345678\x1B[0m, "
    "\x1B[34mbcdef012\x1B[0m, \x1B[34m3456789a\x1B[0m, "
    "\x1B[34mdef01234\x1B[0m, \x1B[34m56789abc\x1B[0m, "
    "\x1B[34mf0123456\x1B[0m, \x1B[34m789abcde\x1B[0m\n"
    "          u16:\x1B[32muint16[]\x1B[0m: "
    "\x1B[34mdef0\x1B[0m, \x1B[34m9abc\x1B[0m, \x1B[34m5678\x1B[0m, \x1B[34m1234\x1B[0m, "
    "\x1B[34mf012\x1B[0m, \x1B[34mbcde\x1B[0m, \x1B[34m789a\x1B[0m, \x1B[34m3456\x1B[0m, "
    "\x1B[34m1234\x1B[0m, \x1B[34mdef0\x1B[0m, \x1B[34m9abc\x1B[0m, \x1B[34m5678\x1B[0m, "
    "\x1B[34m3456\x1B[0m, \x1B[34mf012\x1B[0m, \x1B[34mbcde\x1B[0m, \x1B[34m789a\x1B[0m\n"
    "          u8:\x1B[32muint8[]\x1B[0m: "
    "\x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m,"
    " \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, "
    "\x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m,"
    " \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, "
    "\x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m, \x1B[34mde\x1B[0m,"
    " \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m, \x1B[34m56\x1B[0m, "
    "\x1B[34m56\x1B[0m, \x1B[34m34\x1B[0m, \x1B[34m12\x1B[0m, \x1B[34mf0\x1B[0m,"
    " \x1B[34mde\x1B[0m, \x1B[34mbc\x1B[0m, \x1B[34m9a\x1B[0m, \x1B[34m78\x1B[0m\n"
    "        }\n"
    "      }\n");

void InitSignalOne(zx_port_packet_t* packet) {
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_SIGNAL_ONE;
  packet->status = ZX_OK;
  packet->signal.trigger = __ZX_OBJECT_READABLE | __ZX_OBJECT_PEER_CLOSED;
  packet->signal.observed = __ZX_OBJECT_READABLE | __ZX_OBJECT_WRITABLE;
  packet->signal.count = kSignalCount;
  packet->signal.timestamp = 0;
  packet->signal.reserved1 = 0;
}

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitSignalOne, ZX_OK, kHandle, ZX_TIME_INFINITE, InitSignalOne,
    ("\n"
     "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
     "zx_port_wait("
     "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
     "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
     "  -> \x1B[32mZX_OK\x1B[0m\n"
     "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
     "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
     "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_SIGNAL_ONE\x1B[0m\n"
     "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
     "        signal:\x1B[32mzx_packet_signal_t\x1B[0m: {\n"
     "          trigger:\x1B[32msignals\x1B[0m: \x1B[34m__ZX_OBJECT_READABLE | "
     "__ZX_OBJECT_PEER_CLOSED\x1B[0m\n"
     "          observed:\x1B[32msignals\x1B[0m: \x1B[34m__ZX_OBJECT_READABLE | "
     "__ZX_OBJECT_WRITABLE\x1B[0m\n"
     "          count:\x1B[32muint64\x1B[0m: \x1B[34m2\x1B[0m\n" +
     ClockExpected(
         0, "          timestamp:\x1B[32mtime\x1B[0m: \x1B[34m%c and 000000000 ns\x1B[0m\n") +
     "          reserved1:\x1B[32muint64\x1B[0m: \x1B[34m0\x1B[0m\n"
     "        }\n"
     "      }\n")
        .c_str());

void InitSignalRep(zx_port_packet_t* packet) {
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_SIGNAL_REP;
  packet->status = ZX_OK;
  packet->signal.trigger = __ZX_OBJECT_READABLE | __ZX_OBJECT_PEER_CLOSED;
  packet->signal.observed = __ZX_OBJECT_READABLE | __ZX_OBJECT_WRITABLE;
  packet->signal.count = kSignalCount;
  packet->signal.timestamp = 0;
  packet->signal.reserved1 = 0;
}

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitSignalRep, ZX_OK, kHandle, ZX_TIME_INFINITE, InitSignalRep,
    ("\n"
     "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
     "zx_port_wait("
     "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
     "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
     "  -> \x1B[32mZX_OK\x1B[0m\n"
     "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
     "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
     "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_SIGNAL_REP\x1B[0m\n"
     "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
     "        signal:\x1B[32mzx_packet_signal_t\x1B[0m: {\n"
     "          trigger:\x1B[32msignals\x1B[0m: \x1B[34m__ZX_OBJECT_READABLE | "
     "__ZX_OBJECT_PEER_CLOSED\x1B[0m\n"
     "          observed:\x1B[32msignals\x1B[0m: \x1B[34m__ZX_OBJECT_READABLE | "
     "__ZX_OBJECT_WRITABLE\x1B[0m\n"
     "          count:\x1B[32muint64\x1B[0m: \x1B[34m2\x1B[0m\n" +
     ClockExpected(
         0, "          timestamp:\x1B[32mtime\x1B[0m: \x1B[34m%c and 000000000 ns\x1B[0m\n") +
     "          reserved1:\x1B[32muint64\x1B[0m: \x1B[34m0\x1B[0m\n"
     "        }\n"
     "      }\n")
        .c_str());

void InitException0(zx_port_packet_t* packet) {
  constexpr uint64_t kPid = 7865;
  constexpr uint64_t kTid = 1265;
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_EXCEPTION(0);
  packet->status = ZX_OK;
  packet->exception.pid = kPid;
  packet->exception.tid = kTid;
  packet->exception.reserved0 = 1;
  packet->exception.reserved1 = 2;
}

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitException0, ZX_OK, kHandle, ZX_TIME_INFINITE, InitException0,
    ("\n"
     "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
     "zx_port_wait("
     "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
     "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
     "  -> \x1B[32mZX_OK\x1B[0m\n"
     "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
     "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
     "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_EXCEPTION(0)\x1B[0m\n"
     "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
     "        exception:\x1B[32mzx_packet_exception_t\x1B[0m: {\n"
     "          pid:\x1B[32muint64\x1B[0m: \x1B[34m7865\x1B[0m\n"
     "          tid:\x1B[32muint64\x1B[0m: \x1B[34m1265\x1B[0m\n"
     "          reserved0:\x1B[32muint64\x1B[0m: \x1B[34m1\x1B[0m\n"
     "          reserved1:\x1B[32muint64\x1B[0m: \x1B[34m2\x1B[0m\n"
     "        }\n"
     "      }\n"));

void InitException3(zx_port_packet_t* packet) {
  constexpr uint64_t kPid = 7865;
  constexpr uint64_t kTid = 1265;
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_EXCEPTION(3);
  packet->status = ZX_OK;
  packet->exception.pid = kPid;
  packet->exception.tid = kTid;
  packet->exception.reserved0 = 1;
  packet->exception.reserved1 = 2;
}

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitException3, ZX_OK, kHandle, ZX_TIME_INFINITE, InitException3,
    ("\n"
     "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
     "zx_port_wait("
     "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
     "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
     "  -> \x1B[32mZX_OK\x1B[0m\n"
     "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
     "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
     "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_EXCEPTION(3)\x1B[0m\n"
     "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
     "        exception:\x1B[32mzx_packet_exception_t\x1B[0m: {\n"
     "          pid:\x1B[32muint64\x1B[0m: \x1B[34m7865\x1B[0m\n"
     "          tid:\x1B[32muint64\x1B[0m: \x1B[34m1265\x1B[0m\n"
     "          reserved0:\x1B[32muint64\x1B[0m: \x1B[34m1\x1B[0m\n"
     "          reserved1:\x1B[32muint64\x1B[0m: \x1B[34m2\x1B[0m\n"
     "        }\n"
     "      }\n"));

void InitGuestBell(zx_port_packet_t* packet) {
  constexpr uint64_t kAddr = 0x78654321;
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_GUEST_BELL;
  packet->status = ZX_OK;
  packet->guest_bell.addr = kAddr;
  packet->guest_bell.reserved0 = 0;
  packet->guest_bell.reserved1 = 1;
  packet->guest_bell.reserved2 = 2;
}

PORT_WAIT_DISPLAY_TEST(
    ZxPortWaitGuestBell, ZX_OK, kHandle, ZX_TIME_INFINITE, InitGuestBell,
    ("\n"
     "test_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
     "zx_port_wait("
     "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
     "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
     "  -> \x1B[32mZX_OK\x1B[0m\n"
     "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
     "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
     "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_GUEST_BELL\x1B[0m\n"
     "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
     "        guest_bell:\x1B[32mzx_packet_guest_bell_t\x1B[0m: {\n"
     "          addr:\x1B[32mzx_gpaddr_t\x1B[0m: \x1B[34m0000000078654321\x1B[0m\n"
     "          reserved0:\x1B[32muint64\x1B[0m: \x1B[34m0\x1B[0m\n"
     "          reserved1:\x1B[32muint64\x1B[0m: \x1B[34m1\x1B[0m\n"
     "          reserved2:\x1B[32muint64\x1B[0m: \x1B[34m2\x1B[0m\n"
     "        }\n"
     "      }\n"));

void InitGuestMemX64(zx_port_packet_t* packet) {
  constexpr uint64_t kAddr = 0x78654321;
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_GUEST_MEM;
  packet->status = ZX_OK;
  zx_packet_guest_mem_x86_t mem;
  mem.addr = kAddr;
  mem.inst_len = 3;
  memset(mem.inst_buf, 0, sizeof(mem.inst_buf));
  mem.inst_buf[0] = 1;
  mem.inst_buf[1] = 2;
  mem.inst_buf[2] = 3;
  mem.default_operand_size = 1;
  memset(mem.reserved, 0, sizeof(mem.reserved));
  memcpy(&packet->guest_mem, &mem, sizeof(packet->guest_mem));
}

TEST_F(InterceptionWorkflowTestX64, ZxPortWaitGuestMemX64) {
  PORT_WAIT_DISPLAY_TEST_CONTENT(
      ZX_OK, kHandle, ZX_TIME_INFINITE, InitGuestMemX64,
      "\ntest_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
      "zx_port_wait("
      "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
      "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
      "  -> \x1B[32mZX_OK\x1B[0m\n"
      "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
      "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
      "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_GUEST_MEM\x1B[0m\n"
      "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
      "        guest_mem:\x1B[32mzx_packet_guest_mem_x86_t\x1B[0m: {\n"
      "          addr:\x1B[32mzx_gpaddr_t\x1B[0m: \x1B[34m0000000078654321\x1B[0m\n"
      "          inst_len:\x1B[32muint8\x1B[0m: \x1B[34m3\x1B[0m\n"
      "          inst_buf:\x1B[32muint8[]\x1B[0m: \x1B[34m01\x1B[0m, \x1B[34m02\x1B[0m, "
      "\x1B[34m03\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, "
      "\x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, "
      "\x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, "
      "\x1B[34m00\x1B[0m\n"
      "          default_operand_size:\x1B[32muint8\x1B[0m: \x1B[34m1\x1B[0m\n"
      "          reserved:\x1B[32muint8[]\x1B[0m: \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, "
      "\x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, \x1B[34m00\x1B[0m, "
      "\x1B[34m00\x1B[0m\n"
      "        }\n"
      "      }\n");
}

void InitGuestMemAArch64(zx_port_packet_t* packet) {
  constexpr uint64_t kAddr = 0x78654321;
  packet->key = kKey;
  packet->type = ZX_PKT_TYPE_GUEST_MEM;
  packet->status = ZX_OK;
  zx_packet_guest_mem_aarch64_t mem;
  mem.addr = kAddr;
  mem.access_size = 2;
  mem.sign_extend = false;
  mem.xt = 1;
  mem.read = true;
  constexpr uint64_t kData = 0x13579bdf2468ace0UL;
  mem.data = kData;
  mem.reserved = 0;
  memcpy(&packet->guest_mem, &mem, sizeof(packet->guest_mem));
}

TEST_F(InterceptionWorkflowTestArm, ZxPortWaitGuestMemAArch64) {
  PORT_WAIT_DISPLAY_TEST_CONTENT(
      ZX_OK, kHandle, ZX_TIME_INFINITE, InitGuestMemAArch64,
      "\ntest_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
      "zx_port_wait("
      "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
      "deadline:\x1B[32mtime\x1B[0m: \x1B[34mZX_TIME_INFINITE\x1B[0m)\n"
      "  -> \x1B[32mZX_OK\x1B[0m\n"
      "      packet:\x1B[32mzx_port_packet_t\x1B[0m: {\n"
      "        key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m\n"
      "        type:\x1B[32mzx_port_packet_t::type\x1B[0m: \x1B[34mZX_PKT_TYPE_GUEST_MEM\x1B[0m\n"
      "        status:\x1B[32mstatus_t\x1B[0m: \x1B[32mZX_OK\x1B[0m\n"
      "        guest_mem:\x1B[32mzx_packet_guest_mem_aarch64_t\x1B[0m: {\n"
      "          addr:\x1B[32mzx_gpaddr_t\x1B[0m: \x1B[34m0000000078654321\x1B[0m\n"
      "          access_size:\x1B[32muint8\x1B[0m: \x1B[34m2\x1B[0m\n"
      "          sign_extend:\x1B[32mbool\x1B[0m: \x1B[34mfalse\x1B[0m\n"
      "          xt:\x1B[32muint8\x1B[0m: \x1B[34m1\x1B[0m\n"
      "          read:\x1B[32mbool\x1B[0m: \x1B[34mtrue\x1B[0m\n"
      "          data:\x1B[32muint64\x1B[0m: \x1B[34m1393753992385309920\x1B[0m\n"
      "          reserved:\x1B[32muint64\x1B[0m: \x1B[34m0\x1B[0m\n"
      "        }\n"
      "      }\n");
}

// zx_port_cancel tests.

std::unique_ptr<SystemCallTest> ZxPortCancel(int64_t status, std::string_view status_name,
                                             zx_handle_t handle, zx_handle_t source, uint64_t key) {
  auto value = std::make_unique<SystemCallTest>("zx_port_cancel", status, status_name);
  value->AddInput(handle);
  value->AddInput(source);
  value->AddInput(key);
  return value;
}

#define PORT_CANCEL_DISPLAY_TEST_CONTENT(status, expected)                                        \
  PerformDisplayTest("zx_port_cancel@plt", ZxPortCancel(status, #status, kHandle, kSource, kKey), \
                     expected);

#define PORT_CANCEL_DISPLAY_TEST(name, status, expected) \
  TEST_F(InterceptionWorkflowTestX64, name) {            \
    PORT_CANCEL_DISPLAY_TEST_CONTENT(status, expected);  \
  }                                                      \
  TEST_F(InterceptionWorkflowTestArm, name) { PORT_CANCEL_DISPLAY_TEST_CONTENT(status, expected); }

PORT_CANCEL_DISPLAY_TEST(ZxPortCancel, ZX_OK,
                         "\ntest_3141 \x1B[31m3141\x1B[0m:\x1B[31m8764\x1B[0m "
                         "zx_port_cancel("
                         "handle:\x1B[32mhandle\x1B[0m: \x1B[31mcefa1db0\x1B[0m, "
                         "source:\x1B[32mhandle\x1B[0m: \x1B[31m00ab1234\x1B[0m, "
                         "key:\x1B[32muint64\x1B[0m: \x1B[34m1234\x1B[0m)\n"
                         "  -> \x1B[32mZX_OK\x1B[0m\n");

}  // namespace fidlcat
