// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tools/fidlcat/lib/type_decoder.h"

#include <zircon/system/public/zircon/rights.h>
#include <zircon/system/public/zircon/syscalls/object.h>
#include <zircon/system/public/zircon/syscalls/port.h>
#include <zircon/system/public/zircon/types.h>

#include <cstdint>
#include <iomanip>
#include <ostream>

namespace fidlcat {

#define ClockNameCase(name) \
  case name:                \
    os << #name;            \
    return

void ClockName(zx_clock_t clock, std::ostream& os) {
  switch (clock) {
    ClockNameCase(ZX_CLOCK_MONOTONIC);
    ClockNameCase(ZX_CLOCK_UTC);
    ClockNameCase(ZX_CLOCK_THREAD);
    default:
      os << clock;
      return;
  }
}

#define ObjTypeNameCase(name) \
  case name:                  \
    os << #name;              \
    return

void ObjTypeName(zx_obj_type_t obj_type, std::ostream& os) {
  switch (obj_type) {
    ObjTypeNameCase(ZX_OBJ_TYPE_NONE);
    ObjTypeNameCase(ZX_OBJ_TYPE_PROCESS);
    ObjTypeNameCase(ZX_OBJ_TYPE_THREAD);
    ObjTypeNameCase(ZX_OBJ_TYPE_VMO);
    ObjTypeNameCase(ZX_OBJ_TYPE_CHANNEL);
    ObjTypeNameCase(ZX_OBJ_TYPE_EVENT);
    ObjTypeNameCase(ZX_OBJ_TYPE_PORT);
    ObjTypeNameCase(ZX_OBJ_TYPE_INTERRUPT);
    ObjTypeNameCase(ZX_OBJ_TYPE_PCI_DEVICE);
    ObjTypeNameCase(ZX_OBJ_TYPE_LOG);
    ObjTypeNameCase(ZX_OBJ_TYPE_SOCKET);
    ObjTypeNameCase(ZX_OBJ_TYPE_RESOURCE);
    ObjTypeNameCase(ZX_OBJ_TYPE_EVENTPAIR);
    ObjTypeNameCase(ZX_OBJ_TYPE_JOB);
    ObjTypeNameCase(ZX_OBJ_TYPE_VMAR);
    ObjTypeNameCase(ZX_OBJ_TYPE_FIFO);
    ObjTypeNameCase(ZX_OBJ_TYPE_GUEST);
    ObjTypeNameCase(ZX_OBJ_TYPE_VCPU);
    ObjTypeNameCase(ZX_OBJ_TYPE_TIMER);
    ObjTypeNameCase(ZX_OBJ_TYPE_IOMMU);
    ObjTypeNameCase(ZX_OBJ_TYPE_BTI);
    ObjTypeNameCase(ZX_OBJ_TYPE_PROFILE);
    ObjTypeNameCase(ZX_OBJ_TYPE_PMT);
    ObjTypeNameCase(ZX_OBJ_TYPE_SUSPEND_TOKEN);
    ObjTypeNameCase(ZX_OBJ_TYPE_PAGER);
    ObjTypeNameCase(ZX_OBJ_TYPE_EXCEPTION);
    default:
      os << obj_type;
      return;
  }
}

constexpr uint32_t kExceptionMask = 0xff;
constexpr int kExceptionNumberShift = 8;
constexpr uint32_t kExceptionNumberMask = 0xff;

#define PortPacketTypeNameCase(name) \
  case name:                         \
    os << #name;                     \
    return

void PortPacketTypeName(uint32_t type, std::ostream& os) {
  switch (type) {
    PortPacketTypeNameCase(ZX_PKT_TYPE_USER);
    PortPacketTypeNameCase(ZX_PKT_TYPE_SIGNAL_ONE);
    PortPacketTypeNameCase(ZX_PKT_TYPE_SIGNAL_REP);
    PortPacketTypeNameCase(ZX_PKT_TYPE_GUEST_BELL);
    PortPacketTypeNameCase(ZX_PKT_TYPE_GUEST_MEM);
    PortPacketTypeNameCase(ZX_PKT_TYPE_GUEST_IO);
    PortPacketTypeNameCase(ZX_PKT_TYPE_GUEST_VCPU);
    PortPacketTypeNameCase(ZX_PKT_TYPE_INTERRUPT);
    PortPacketTypeNameCase(ZX_PKT_TYPE_PAGE_REQUEST);
    default:
      if ((type & kExceptionMask) == ZX_PKT_TYPE_EXCEPTION(0)) {
        os << "ZX_PKT_TYPE_EXCEPTION(" << ((type >> kExceptionNumberShift) & kExceptionNumberMask)
           << ')';
        return;
      }
      os << "port_packet_type=" << type;
      return;
  }
}

#define RightsNameCase(name)    \
  if ((rights & (name)) != 0) { \
    os << separator << #name;   \
    separator = " | ";          \
  }

void RightsName(zx_rights_t rights, std::ostream& os) {
  if (rights == 0) {
    os << "ZX_RIGHT_NONE";
    return;
  }
  const char* separator = "";
  RightsNameCase(ZX_RIGHT_DUPLICATE);
  RightsNameCase(ZX_RIGHT_TRANSFER);
  RightsNameCase(ZX_RIGHT_READ);
  RightsNameCase(ZX_RIGHT_WRITE);
  RightsNameCase(ZX_RIGHT_EXECUTE);
  RightsNameCase(ZX_RIGHT_MAP);
  RightsNameCase(ZX_RIGHT_GET_PROPERTY);
  RightsNameCase(ZX_RIGHT_SET_PROPERTY);
  RightsNameCase(ZX_RIGHT_ENUMERATE);
  RightsNameCase(ZX_RIGHT_DESTROY);
  RightsNameCase(ZX_RIGHT_SET_POLICY);
  RightsNameCase(ZX_RIGHT_GET_POLICY);
  RightsNameCase(ZX_RIGHT_SIGNAL);
  RightsNameCase(ZX_RIGHT_SIGNAL_PEER);
  RightsNameCase(ZX_RIGHT_WAIT);
  RightsNameCase(ZX_RIGHT_INSPECT);
  RightsNameCase(ZX_RIGHT_MANAGE_JOB);
  RightsNameCase(ZX_RIGHT_MANAGE_PROCESS);
  RightsNameCase(ZX_RIGHT_MANAGE_THREAD);
  RightsNameCase(ZX_RIGHT_APPLY_PROFILE);
  RightsNameCase(ZX_RIGHT_SAME_RIGHTS);
}

#define StatusNameCase(name) \
  case name:                 \
    os << #name;             \
    return

void StatusName(zx_status_t status, std::ostream& os) {
  switch (status) {
    StatusNameCase(ZX_OK);
    StatusNameCase(ZX_ERR_INTERNAL);
    StatusNameCase(ZX_ERR_NOT_SUPPORTED);
    StatusNameCase(ZX_ERR_NO_RESOURCES);
    StatusNameCase(ZX_ERR_NO_MEMORY);
    StatusNameCase(ZX_ERR_INTERNAL_INTR_RETRY);
    StatusNameCase(ZX_ERR_INVALID_ARGS);
    StatusNameCase(ZX_ERR_BAD_HANDLE);
    StatusNameCase(ZX_ERR_WRONG_TYPE);
    StatusNameCase(ZX_ERR_BAD_SYSCALL);
    StatusNameCase(ZX_ERR_OUT_OF_RANGE);
    StatusNameCase(ZX_ERR_BUFFER_TOO_SMALL);
    StatusNameCase(ZX_ERR_BAD_STATE);
    StatusNameCase(ZX_ERR_TIMED_OUT);
    StatusNameCase(ZX_ERR_SHOULD_WAIT);
    StatusNameCase(ZX_ERR_CANCELED);
    StatusNameCase(ZX_ERR_PEER_CLOSED);
    StatusNameCase(ZX_ERR_NOT_FOUND);
    StatusNameCase(ZX_ERR_ALREADY_EXISTS);
    StatusNameCase(ZX_ERR_ALREADY_BOUND);
    StatusNameCase(ZX_ERR_UNAVAILABLE);
    StatusNameCase(ZX_ERR_ACCESS_DENIED);
    StatusNameCase(ZX_ERR_IO);
    StatusNameCase(ZX_ERR_IO_REFUSED);
    StatusNameCase(ZX_ERR_IO_DATA_INTEGRITY);
    StatusNameCase(ZX_ERR_IO_DATA_LOSS);
    StatusNameCase(ZX_ERR_IO_NOT_PRESENT);
    StatusNameCase(ZX_ERR_IO_OVERRUN);
    StatusNameCase(ZX_ERR_IO_MISSED_DEADLINE);
    StatusNameCase(ZX_ERR_IO_INVALID);
    StatusNameCase(ZX_ERR_BAD_PATH);
    StatusNameCase(ZX_ERR_NOT_DIR);
    StatusNameCase(ZX_ERR_NOT_FILE);
    StatusNameCase(ZX_ERR_FILE_BIG);
    StatusNameCase(ZX_ERR_NO_SPACE);
    StatusNameCase(ZX_ERR_NOT_EMPTY);
    StatusNameCase(ZX_ERR_STOP);
    StatusNameCase(ZX_ERR_NEXT);
    StatusNameCase(ZX_ERR_ASYNC);
    StatusNameCase(ZX_ERR_PROTOCOL_NOT_SUPPORTED);
    StatusNameCase(ZX_ERR_ADDRESS_UNREACHABLE);
    StatusNameCase(ZX_ERR_ADDRESS_IN_USE);
    StatusNameCase(ZX_ERR_NOT_CONNECTED);
    StatusNameCase(ZX_ERR_CONNECTION_REFUSED);
    StatusNameCase(ZX_ERR_CONNECTION_RESET);
    StatusNameCase(ZX_ERR_CONNECTION_ABORTED);
    default:
      os << "status=" << status;
      return;
  }
}

void StatusName(const Colors& colors, zx_status_t status, std::ostream& os) {
  if (status == ZX_OK) {
    os << colors.green;
  } else {
    os << colors.red;
  }
  StatusName(status, os);
  os << colors.reset;
}

#define SignalNameCase(name)          \
  if ((signals & (name)) == (name)) { \
    os << separator << #name;         \
    separator = " | ";                \
  }

void SignalName(zx_signals_t signals, std::ostream& os) {
  if (signals == 0) {
    os << "0";
    return;
  }
  if (signals == __ZX_OBJECT_SIGNAL_ALL) {
    os << "__ZX_OBJECT_SIGNAL_ALL";
    return;
  }
  const char* separator = "";
  SignalNameCase(__ZX_OBJECT_READABLE);
  SignalNameCase(__ZX_OBJECT_WRITABLE);
  SignalNameCase(__ZX_OBJECT_PEER_CLOSED);
  SignalNameCase(__ZX_OBJECT_SIGNALED);
  SignalNameCase(__ZX_OBJECT_SIGNAL_4);
  SignalNameCase(__ZX_OBJECT_SIGNAL_5);
  SignalNameCase(__ZX_OBJECT_SIGNAL_6);
  SignalNameCase(__ZX_OBJECT_SIGNAL_7);
  SignalNameCase(__ZX_OBJECT_SIGNAL_8);
  SignalNameCase(__ZX_OBJECT_SIGNAL_9);
  SignalNameCase(__ZX_OBJECT_SIGNAL_10);
  SignalNameCase(__ZX_OBJECT_SIGNAL_11);
  SignalNameCase(__ZX_OBJECT_SIGNAL_12);
  SignalNameCase(__ZX_OBJECT_SIGNAL_13);
  SignalNameCase(__ZX_OBJECT_SIGNAL_14);
  SignalNameCase(__ZX_OBJECT_SIGNAL_15);
  SignalNameCase(__ZX_OBJECT_SIGNAL_16);
  SignalNameCase(__ZX_OBJECT_SIGNAL_17);
  SignalNameCase(__ZX_OBJECT_SIGNAL_18);
  SignalNameCase(__ZX_OBJECT_SIGNAL_19);
  SignalNameCase(__ZX_OBJECT_SIGNAL_20);
  SignalNameCase(__ZX_OBJECT_SIGNAL_21);
  SignalNameCase(__ZX_OBJECT_SIGNAL_22);
  SignalNameCase(__ZX_OBJECT_HANDLE_CLOSED);
  SignalNameCase(ZX_USER_SIGNAL_0);
  SignalNameCase(ZX_USER_SIGNAL_1);
  SignalNameCase(ZX_USER_SIGNAL_2);
  SignalNameCase(ZX_USER_SIGNAL_3);
  SignalNameCase(ZX_USER_SIGNAL_4);
  SignalNameCase(ZX_USER_SIGNAL_5);
  SignalNameCase(ZX_USER_SIGNAL_6);
  SignalNameCase(ZX_USER_SIGNAL_7);
}

constexpr int kUint32Precision = 8;

void DisplayHandle(const Colors& colors, const zx_handle_info_t& handle, std::ostream& os) {
  os << colors.red;
  if (handle.type != ZX_OBJ_TYPE_NONE) {
    ObjTypeName(handle.type, os);
    os << ':';
  }
  os << std::hex << std::setfill('0') << std::setw(kUint32Precision) << handle.handle << std::dec
     << std::setw(0);
  if (handle.rights != 0) {
    os << colors.blue << '(';
    RightsName(handle.rights, os);
    os << ')';
  }
  os << colors.reset;
}

void DisplayType(const Colors& colors, SyscallType type, std::ostream& os) {
  switch (type) {
    case SyscallType::kBool:
      os << ":" << colors.green << "bool" << colors.reset << ": ";
      break;
    case SyscallType::kInt64:
      os << ":" << colors.green << "int64" << colors.reset << ": ";
      break;
    case SyscallType::kUint8:
      os << ":" << colors.green << "uint8" << colors.reset << ": ";
      break;
    case SyscallType::kUint8Array:
      os << ":" << colors.green << "uint8[]" << colors.reset << ": ";
      break;
    case SyscallType::kUint16:
      os << ":" << colors.green << "uint16" << colors.reset << ": ";
      break;
    case SyscallType::kUint16Array:
      os << ":" << colors.green << "uint16[]" << colors.reset << ": ";
      break;
    case SyscallType::kUint32:
      os << ":" << colors.green << "uint32" << colors.reset << ": ";
      break;
    case SyscallType::kUint32Array:
      os << ":" << colors.green << "uint32[]" << colors.reset << ": ";
      break;
    case SyscallType::kUint64:
      os << ":" << colors.green << "uint64" << colors.reset << ": ";
      break;
    case SyscallType::kUint64Array:
      os << ":" << colors.green << "uint64[]" << colors.reset << ": ";
      break;
    case SyscallType::kClock:
      os << ":" << colors.green << "clock" << colors.reset << ": ";
      break;
    case SyscallType::kDuration:
      os << ":" << colors.green << "duration" << colors.reset << ": ";
      break;
    case SyscallType::kGpAddr:
      os << ":" << colors.green << "zx_gpaddr_t" << colors.reset << ": ";
      break;
    case SyscallType::kHandle:
      os << ":" << colors.green << "handle" << colors.reset << ": ";
      break;
    case SyscallType::kPortPacketType:
      os << ":" << colors.green << "zx_port_packet_t::type" << colors.reset << ": ";
      break;
    case SyscallType::kSignals:
      os << ":" << colors.green << "signals" << colors.reset << ": ";
      break;
    case SyscallType::kStatus:
      os << ":" << colors.green << "status_t" << colors.reset << ": ";
      break;
    case SyscallType::kTime:
      os << ":" << colors.green << "time" << colors.reset << ": ";
      break;
    default:
      os << ":" << colors.green << "unimplemented type " << static_cast<uint32_t>(type)
         << colors.reset;
      return;
  }
}

}  // namespace fidlcat
