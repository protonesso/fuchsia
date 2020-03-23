// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SRC_MEDIA_AUDIO_AUDIO_CORE_TESTING_FAKE_AUDIO_DRIVER_H_
#define SRC_MEDIA_AUDIO_AUDIO_CORE_TESTING_FAKE_AUDIO_DRIVER_H_

#include <fuchsia/hardware/audio/cpp/fidl.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/async/cpp/time.h>
#include <lib/fzl/vmo-mapper.h>
#include <lib/zx/channel.h>
#include <lib/zx/vmo.h>
#include <zircon/device/audio.h>

#include <cstring>
#include <optional>

#include "src/media/audio/lib/test/message_transceiver.h"

namespace media::audio::testing {

namespace audio_fidl = ::fuchsia::hardware::audio;

class FakeAudioDriver : public audio_fidl::StreamConfig,
                        public audio_fidl::RingBuffer {
 public:
  FakeAudioDriver(zx::channel channel, async_dispatcher_t* dispatcher);

  fzl::VmoMapper CreateRingBufferFakeDriver(size_t size);
  void StartFakeDriver();
  void StopFakeDriver();

  void set_stream_unique_id(const audio_stream_unique_id_t& uid) {
    std::memcpy(uid_.data, uid.data, sizeof(uid.data));
  }
  void set_device_manufacturer(std::string mfgr) { manufacturer_ = std::move(mfgr); }
  void set_device_product(std::string product) { product_ = std::move(product); }
  void set_gain(float gain) { cur_gain_ = gain; }
  void set_gain_limits(float min_gain, float max_gain) {
    gain_limits_ = std::make_pair(min_gain, max_gain);
  }
  void set_can_agc(bool can_agc) { can_agc_ = can_agc; }
  void set_cur_agc(bool cur_agc) { cur_agc_ = cur_agc; }
  void set_can_mute(bool can_mute) { can_mute_ = can_mute; }
  void set_cur_mute(bool cur_mute) { cur_mute_ = cur_mute; }
  void set_formats(audio_fidl::PcmSupportedFormats formats) {
    formats_ = std::move(formats);
  }
  void set_clock_domain(int32_t clock_domain) { clock_domain_ = clock_domain; }
  void set_plugged(bool plugged) { plugged_ = plugged; }
  void set_fifo_depth(uint32_t fifo_depth) { fifo_depth_ = fifo_depth; }

  // |true| after an |audio_rb_cmd_start| is received, until an |audio_rb_cmd_stop| is received.
  bool is_running() const { return is_running_; }

  // The 'selected format' for the driver.
  // The returned optional will be empty if no |CreateRingBuffer| command has been received.
  std::optional<audio_fidl::PcmFormat> selected_format() const { return selected_format_; }

 private:
  void OnInboundStreamMessage(test::MessageTransceiver::Message message);
  void OnInboundStreamError(zx_status_t status);
  void HandleCommandGetUniqueId(const audio_stream_cmd_get_unique_id_req_t& request);
  void HandleCommandGetString(const audio_stream_cmd_get_string_req_t& request);
  void HandleCommandGetGain(const audio_stream_cmd_get_gain_req_t& request);
  void HandleCommandSetFormat(const audio_stream_cmd_set_format_req_t& request);
  void HandleCommandPlugDetect(const audio_stream_cmd_plug_detect_req_t& request);
  void HandleCommandGetClockDomain(const audio_stream_cmd_get_clock_domain_req_t& request);

  void OnInboundRingBufferMessage(test::MessageTransceiver::Message message);
  void OnInboundRingBufferError(zx_status_t status);
  void HandleCommandGetFifoDepth(audio_rb_cmd_get_fifo_depth_req_t& request);
  void HandleCommandGetBuffer(audio_rb_cmd_get_buffer_req_t& request);
  void HandleCommandStart(audio_rb_cmd_start_req_t& request);
  void HandleCommandStop(audio_rb_cmd_stop_req_t& request);

  // fuchsia hardware audio StreamConfig Interface
  void GetProperties(audio_fidl::StreamConfig::GetPropertiesCallback callback) override;
  void GetSupportedFormats(audio_fidl::StreamConfig::GetSupportedFormatsCallback callback) override;
  void CreateRingBuffer(audio_fidl::Format format,
                        ::fidl::InterfaceRequest<audio_fidl::RingBuffer> ring_buffer) override;
  void WatchGainState(audio_fidl::StreamConfig::WatchGainStateCallback callback) override;
  void SetGain(audio_fidl::GainState target_state) override;
  void WatchPlugState(audio_fidl::StreamConfig::WatchPlugStateCallback callback) override;

  // fuchsia hardware audio RingBuffer Interface
  void GetProperties(audio_fidl::RingBuffer::GetPropertiesCallback callback) override;
  void WatchClockRecoveryPositionInfo(
    audio_fidl::RingBuffer::WatchClockRecoveryPositionInfoCallback callback) override;
  void GetVmo(uint32_t min_frames, uint32_t clock_recovery_notifications_per_ring,
              audio_fidl::RingBuffer::GetVmoCallback callback) override;
  void Start(audio_fidl::RingBuffer::StartCallback callback) override;
  void Stop(audio_fidl::RingBuffer::StopCallback callback) override;

  audio_stream_unique_id_t uid_ = {};
  std::string manufacturer_ = "default manufacturer";
  std::string product_ = "default product";
  float cur_gain_ = 0.0f;
  std::pair<float, float> gain_limits_{-160.0f, 3.0f};
  bool can_agc_ = true;
  bool cur_agc_ = false;
  bool can_mute_ = true;
  bool cur_mute_ = false;
  audio_fidl::PcmSupportedFormats formats_ = {};
  int32_t clock_domain_ = 0;
  size_t ring_buffer_size_;
  zx::vmo ring_buffer_;

  uint32_t fifo_depth_ = 0;
  bool plugged_ = true;

  std::optional<audio_fidl::PcmFormat> selected_format_;

  bool is_running_ = false;

  async_dispatcher_t* dispatcher_;
  fidl::Binding<audio_fidl::StreamConfig> stream_binding_;
  std::optional<fidl::Binding<audio_fidl::RingBuffer>> ring_buffer_binding_;
  fidl::InterfaceRequest <audio_fidl::StreamConfig> stream_req_;
//  fidl::InterfaceRequest <audio_fidl::RingBuffer> ring_buffer_req_;
  bool stopped = false;
};

}  // namespace media::audio::testing

#endif  // SRC_MEDIA_AUDIO_AUDIO_CORE_TESTING_FAKE_AUDIO_DRIVER_H_
