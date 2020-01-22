// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/media/audio/audio_core/testing/fake_audio_driver.h"

#include <audio-proto-utils/format-utils.h>
#include <gtest/gtest.h>

#include "src/lib/syslog/cpp/logger.h"

namespace media::audio::testing {

FakeAudioDriver::FakeAudioDriver(zx::channel channel, async_dispatcher_t* dispatcher) :
  dispatcher_(dispatcher),
  stream_binding_(this, std::move(channel), dispatcher) {
  StopFakeDriver();
  formats_.number_of_channels.push_back(1);
  formats_.sample_formats.push_back(audio_fidl::SampleFormat::PCM_SIGNED);
  formats_.bytes_per_sample.push_back(2);
  formats_.valid_bits_per_sample.push_back(16);
  formats_.frame_rates.push_back(48000);
}

void FakeAudioDriver::GetProperties(audio_fidl::StreamConfig::GetPropertiesCallback callback) {
  audio_fidl::StreamProperties props = {};

  std::memcpy(props.mutable_unique_id()->data(), uid_.data, sizeof(uid_.data));
  *props.mutable_manufacturer() = manufacturer_;
  *props.mutable_product() = product_;
  *props.mutable_can_mute() = can_mute_;
  *props.mutable_can_agc() = can_agc_;
  *props.mutable_min_gain_db() = gain_limits_.first;
  *props.mutable_max_gain_db() = gain_limits_.second;
  *props.mutable_gain_step_db() = 0.001f;

  callback(std::move(props));
}

void FakeAudioDriver::GetSupportedFormats(
  audio_fidl::StreamConfig::GetSupportedFormatsCallback callback) {
  audio_fidl::SupportedFormats formats = {};
  *formats.mutable_pcm_supported_formats() = formats_;

  std::vector<audio_fidl::SupportedFormats> all_formats = {};
  all_formats.push_back(std::move(formats));
  callback(std::move(all_formats));
}

void FakeAudioDriver::CreateRingBuffer(
  audio_fidl::Format format,
  ::fidl::InterfaceRequest<audio_fidl::RingBuffer> ring_buffer) {
  ring_buffer_binding_.emplace(this, ring_buffer.TakeChannel(), dispatcher_);
  selected_format_ = format.pcm_format();
}

void FakeAudioDriver::WatchGainState(audio_fidl::StreamConfig::WatchGainStateCallback callback) {
  audio_fidl::GainState gain_state = {};
  *gain_state.mutable_muted() = cur_mute_;
  *gain_state.mutable_agc_enabled() = cur_agc_;
  *gain_state.mutable_gain_db() = cur_gain_;
  callback(std::move(gain_state));
}

void FakeAudioDriver::SetGain(audio_fidl::GainState target_state) {
}

void FakeAudioDriver::WatchPlugState(audio_fidl::StreamConfig::WatchPlugStateCallback callback) {
  audio_fidl::PlugState plug_state = {};
  *plug_state.mutable_plugged() = false;
  *plug_state.mutable_plug_state_time() = 0;
  callback(std::move(plug_state));
}

void FakeAudioDriver::StartFakeDriver() {
  assert(!stream_binding_.is_bound());
  stream_binding_.Bind(std::move(stream_req_), dispatcher_);
  // TODO ringbuffer binding
}

void FakeAudioDriver::StopFakeDriver() {
  if (stream_binding_.is_bound()) {
    stream_req_ = stream_binding_.Unbind();
  }
  // TODO ringbuffer unbinding
}

fzl::VmoMapper FakeAudioDriver::CreateRingBufferFakeDriver(size_t size) {
  FX_CHECK(!ring_buffer_) << "Calling CreateRingBuffer multiple times is not supported";

  ring_buffer_size_ = size;
  fzl::VmoMapper mapper;
  mapper.CreateAndMap(ring_buffer_size_, ZX_VM_PERM_READ | ZX_VM_PERM_WRITE, nullptr,
                      &ring_buffer_);
  return mapper;
}

void FakeAudioDriver::GetProperties(audio_fidl::RingBuffer::GetPropertiesCallback callback) {
  audio_fidl::RingBufferProperties props = {};

  *props.mutable_fifo_depth() = fifo_depth_;
  *props.mutable_clock_domain() = clock_domain_;

  callback(std::move(props));
}

void FakeAudioDriver::WatchClockRecoveryPositionInfo(
  audio_fidl::RingBuffer::WatchClockRecoveryPositionInfoCallback callback) {
}

void FakeAudioDriver::GetVmo(uint32_t min_frames, uint32_t clock_recovery_notifications_per_ring,
              audio_fidl::RingBuffer::GetVmoCallback callback) {

  // This should be true since it's set as part of creating the channel that's carrying these
  // messages.
  FX_CHECK(selected_format_);

  if (!ring_buffer_) {
    // If we haven't created a ring buffer, we'll just drop this request.
    return;
  }
  FX_CHECK(ring_buffer_);

  // Dup our ring buffer VMO to send over the channel.
  zx::vmo dup;
  FX_CHECK(ring_buffer_.duplicate(ZX_RIGHT_SAME_RIGHTS, &dup) == ZX_OK);

  // Compute the buffer size in frames.
  auto frame_size = selected_format_->number_of_channels * selected_format_->bytes_per_sample;
  auto ring_buffer_frames = ring_buffer_size_ / frame_size;

  audio_fidl::RingBuffer_GetVmo_Result result = {};
  audio_fidl::RingBuffer_GetVmo_Response response = {};
  response.num_frames = ring_buffer_frames;
  response.ring_buffer = std::move(dup);
  result.set_response(std::move(response));
  callback(std::move(result));
}

void FakeAudioDriver::Start(audio_fidl::RingBuffer::StartCallback callback) {
  EXPECT_TRUE(!is_running_);
  is_running_ = true;
  callback(async::Now(dispatcher_).get());
}

void FakeAudioDriver::Stop(audio_fidl::RingBuffer::StopCallback callback) {
  EXPECT_TRUE(is_running_);
  is_running_ = false;
  callback();
}

}  // namespace media::audio::testing
