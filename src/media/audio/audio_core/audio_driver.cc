// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be found in the LICENSE file.

#include "src/media/audio/audio_core/audio_driver.h"

#include <lib/async/cpp/time.h>
#include <lib/fidl/cpp/clone.h>
#include <lib/zx/clock.h>
#include <string.h>
#include <zircon/status.h>

#include <algorithm>
#include <cstdio>
#include <iomanip>

#include <audio-proto-utils/format-utils.h>
#include <trace/event.h>

#include "src/media/audio/audio_core/driver_utils.h"
#include "src/media/audio/lib/logging/logging.h"

namespace media::audio {
namespace {

static constexpr zx_txid_t TXID = 1;

static constexpr bool kEnablePositionNotifications = false;
// To what extent should position notification messages be logged? If logging level is SPEW, every
// notification is logged (specified by Spew const). If TRACE, log less frequently, specified by
// Trace const. If INFO, even less frequently per Info const (INFO is default for DEBUG builds).
// Default for audio_core in NDEBUG builds is WARNING, so by default we do not log any of these
// messages on Release builds. Set to false to not log at all, even for unsolicited notifications.
static constexpr bool kLogPositionNotifications = false;
static constexpr uint16_t kPositionNotificationSpewInterval = 1;
static constexpr uint16_t kPositionNotificationTraceInterval = 60;
static constexpr uint16_t kPositionNotificationInfoInterval = 3600;

// TODO(39092): Log a cobalt metric for this.
void LogMissedCommandDeadline(zx::duration delay) {
  FX_LOGS(WARNING) << "Driver command missed deadline by " << delay.to_nsecs() << "ns";
}

TimelineFunction TransposeFractionalFramesToBytes(const Format& format,
                                                  TimelineFunction clock_mono_to_fractional_frame) {
  TimelineRate frac_frames_to_bytes(format.bytes_per_frame(),
                                    FractionalFrames<int32_t>(1).raw_value());
  return TimelineFunction::Compose(TimelineFunction(frac_frames_to_bytes),
                                   clock_mono_to_fractional_frame);
}

}  // namespace

AudioDriver::AudioDriver(AudioDevice* owner) : AudioDriver(owner, LogMissedCommandDeadline) {}

AudioDriver::AudioDriver(AudioDevice* owner, DriverTimeoutHandler timeout_handler)
    : owner_(owner),
      timeout_handler_(std::move(timeout_handler)),
      clock_mono_to_fractional_frame_(fbl::MakeRefCounted<VersionedTimelineFunction>()) {
  FX_DCHECK(owner_ != nullptr);
}

zx_status_t AudioDriver::Init(zx::channel stream_channel) {
  TRACE_DURATION("audio", "AudioDriver::Init");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
  FX_DCHECK(state_ == State::Uninitialized);

  // Fetch the KOID of our stream channel. We use this unique ID as our device's device token.
  zx_status_t res;
  zx_info_handle_basic_t sc_info;
  res = stream_channel.get_info(ZX_INFO_HANDLE_BASIC, &sc_info, sizeof(sc_info), nullptr, nullptr);
  if (res != ZX_OK) {
    FX_PLOGS(ERROR, res) << "Failed to to fetch stream channel KOID";
    return res;
  }
  stream_channel_koid_ = sc_info.koid;

  stream_config_intf_ =
      fidl::InterfaceHandle<audio_fidl::StreamConfig>(std::move(stream_channel)).Bind();
  if (!stream_config_intf_.is_bound()) {
    FX_LOGS(ERROR) << "Failed to get stream channel";
    return ZX_ERR_INTERNAL;
  }
  stream_config_intf_.set_error_handler([](zx_status_t status) -> void {
    FX_PLOGS(ERROR, status) << "AudioDriver failed with error: " << status;
  });

  cmd_timeout_.set_handler([this] {
    OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
    DriverCommandTimedOut();
  });

  // We are now initialized, but we don't know any fundamental driver level info, such as:
  //
  // 1) This device's persistent unique ID.
  // 2) The list of formats supported by this device.
  // 3) The user-visible strings for this device (manufacturer, product, etc...).
  state_ = State::MissingDriverInfo;
  return ZX_OK;
}

void AudioDriver::Cleanup() {
  TRACE_DURATION("audio", "AudioDriver::Cleanup");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
  std::shared_ptr<RingBuffer> ring_buffer;
  {
    std::lock_guard<std::mutex> lock(ring_buffer_state_lock_);
    ring_buffer = std::move(ring_buffer_);
  }
  clock_mono_to_fractional_frame_->Update(TimelineFunction());
  ring_buffer = nullptr;

  ring_buffer_channel_wait_.Cancel();
  cmd_timeout_.Cancel();
}

TimelineFunction AudioDriver::clock_mono_to_ring_pos_bytes() const {
  auto format = GetFormat();
  if (format) {
    auto [clock_mono_to_fractional_frame, _] = clock_mono_to_fractional_frame_->get();
    return TransposeFractionalFramesToBytes(*format, clock_mono_to_fractional_frame);
  } else {
    return TimelineFunction();
  }
}

std::optional<Format> AudioDriver::GetFormat() const {
  TRACE_DURATION("audio", "AudioDriver::GetFormat");
  std::lock_guard<std::mutex> lock(configured_format_lock_);
  return configured_format_;
}

zx_status_t AudioDriver::GetDriverInfo() {
  TRACE_DURATION("audio", "AudioDriver::GetDriverInfo");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());

  // We have to be operational in order to fetch supported formats.
  if (!operational()) {
    FX_LOGS(ERROR) << "Cannot fetch supported formats while non-operational (state = "
                   << static_cast<uint32_t>(state_) << ")";
    return ZX_ERR_BAD_STATE;
  }

  // If already fetching initial driver info, get out now and inform our owner when this completes.
  if (fetching_driver_info()) {
    return ZX_OK;
  }

  // Send the commands to get:
  // - persistent unique ID.
  // - manufacturer string.
  // - product string.
  // - gain capabilities.
  // - current gain state.
  // - supported format list.
  // - clock domain.

  // Get unique IDs, strings and gain capabilites.
  stream_config_intf_->GetProperties([this](audio_fidl::StreamProperties props) {
printf("%s oooooooooooooooooo\n", __PRETTY_FUNCTION__);
    OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
    if (state_ != State::MissingDriverInfo) {
      FX_LOGS(ERROR) << "Bad state (" << static_cast<uint32_t>(state_)
                     << ") while handling get string response.";
      ShutdownSelf("Bad state.", ZX_ERR_INTERNAL);
    }
    hw_gain_state_.can_mute = props.has_can_mute() && props.can_mute();
    hw_gain_state_.can_agc = props.has_can_agc() && props.can_agc();
    hw_gain_state_.min_gain = props.min_gain_db();
    hw_gain_state_.max_gain = props.max_gain_db();
    hw_gain_state_.gain_step = props.gain_step_db();
    if (props.has_manufacturer()) {
      manufacturer_name_ = props.manufacturer();
    }
    if (props.has_product()) {
      product_name_ = props.product();
    }
    auto res = OnDriverInfoFetched(kDriverInfoHasUniqueId | kDriverInfoHasMfrStr |
                                   kDriverInfoHasProdStr);
    if (res != ZX_OK) {
      ShutdownSelf("Failed to update info fetched.", res);
    }
  });

  // Get current gain state.
  // We only fetch once per OnDriverInfoFetched, the we are guaranteed by the
  // audio driver interface definition that the driver will reply to the first watch request, we
  // can get the gain state by issuing a watch FIDL call.
  stream_config_intf_->WatchGainState([this](audio_fidl::GainState state) {
    OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
    hw_gain_state_.cur_mute = state.has_muted() && state.muted();
    hw_gain_state_.cur_agc = state.has_agc_enabled() && state.agc_enabled();
    hw_gain_state_.cur_gain = state.gain_db();
    auto res = OnDriverInfoFetched(kDriverInfoHasGainState);
    if (res != ZX_OK) {
      ShutdownSelf("Failed to update info fetched.", res);
    }
  });

  // Get list of supported formats.
  stream_config_intf_->GetSupportedFormats(
    [this](std::vector<audio_fidl::SupportedFormats> formats) {
      OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
      for (auto& i : formats) {
        formats_.emplace_back(i.pcm_supported_formats());
      }
      // Record that we have fetched our format list. This will transition us to Unconfigured
      // state and let our owner know if we are done fetching all the initial driver info needed
      // to operate.
      auto res = OnDriverInfoFetched(kDriverInfoHasFormats);
      if (res != ZX_OK) {
        ShutdownSelf("Failed to update info fetched.", res);
      }
    });

  // Setup command timeout.
  fetch_driver_info_deadline_ =
      async::Now(owner_->mix_domain().dispatcher()) + kDefaultShortCmdTimeout;
  SetupCommandTimeout();
  return ZX_OK;
}

bool FidlSampleFormatIsCompatible(const fuchsia::media::AudioSampleFormat& stream_format,
                                  const audio_fidl::PcmSupportedFormats& supported_formats) {
  struct X {
    audio_fidl::SampleFormat sample_format;
    uint8_t bytes_per_sample;
    uint8_t valid_bits_per_sample;
  };
  using A = fuchsia::media::AudioSampleFormat;
  std::map<fuchsia::media::AudioSampleFormat, X> map = {
    {A::UNSIGNED_8,      {audio_fidl::SampleFormat::PCM_UNSIGNED, 1, 8 }},
    {A::SIGNED_16,       {audio_fidl::SampleFormat::PCM_SIGNED,   2, 16}},
    {A::SIGNED_24_IN_32, {audio_fidl::SampleFormat::PCM_SIGNED,   4, 24}},
    {A::FLOAT,           {audio_fidl::SampleFormat::PCM_FLOAT,    4, 32}},
  };

  auto x = map[stream_format];
  auto& f = supported_formats.sample_formats;

  if (std::find(f.begin(), f.end(), x.sample_format) == f.end()) {
    return false;
  }
  auto& b = supported_formats.bytes_per_sample;
  if (std::find(b.begin(), b.end(), x.bytes_per_sample) == b.end()) {
    return false;
  }
  auto v = supported_formats.valid_bits_per_sample;
  if (std::find(v.begin(), v.end(), x.valid_bits_per_sample) == v.end()) {
    return false;
  }
  return true;
}

bool FidlFormatIsCompatible(const fuchsia::media::AudioStreamType& stream_type,
                            const std::vector<audio_fidl::PcmSupportedFormats>& supported_formats) {
  // Check number of channels.
  auto& ch = supported_formats[0].number_of_channels;
  if (std::find(ch.begin(), ch.end(), stream_type.channels) == ch.end()) return false;

  // Check sample format.
  if (!FidlSampleFormatIsCompatible(stream_type.sample_format, supported_formats[0])) {
    return false;
  }

  // Check the frame rate.
  auto& rates = supported_formats[0].frame_rates;
  if (std::find(rates.begin(), rates.end(), stream_type.frames_per_second) == rates.end()) {
    return false;
  }

  return true;
}

zx_status_t AudioDriver::Configure(const Format& format, zx::duration min_ring_buffer_duration) {
  TRACE_DURATION("audio", "AudioDriver::Configure");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());

  uint32_t channels = format.channels();
  uint32_t frames_per_second = format.frames_per_second();
  fuchsia::media::AudioSampleFormat sample_format = format.sample_format();

  // Sanity check arguments.
  if (channels > std::numeric_limits<uint16_t>::max()) {
    FX_LOGS(ERROR) << "Bad channel count: " << channels;
    return ZX_ERR_INVALID_ARGS;
  }

  // TODO(MTWN-386): sanity check the min_ring_buffer_duration.

  // Check our known format list for compatibility.
  if (!FidlFormatIsCompatible(format.stream_type(), formats_)) {
    FX_LOGS(ERROR) << "No compatible format found when setting format to "
                   << frames_per_second << " Hz " << channels << " Ch Fmt 0x" << std::hex
                   << static_cast<uint32_t>(sample_format);
    return ZX_ERR_INVALID_ARGS;
  }

  // We must be in Unconfigured state to change formats.
  // TODO(MTWN-387): Also permit this if we are in Configured state.
  if (state_ != State::Unconfigured) {
    FX_LOGS(ERROR) << "Bad state while attempting to configure for " << frames_per_second << " Hz "
                   << channels << " Ch Fmt 0x" << std::hex << static_cast<uint32_t>(sample_format)
                   << " (state = " << static_cast<uint32_t>(state_) << ")";
    return ZX_ERR_BAD_STATE;
  }

  // Record the details of our intended target format
  min_ring_buffer_duration_ = min_ring_buffer_duration;
  {
    std::lock_guard<std::mutex> lock(configured_format_lock_);
    configured_format_ = {format};
  }

  zx::channel local_channel;
  zx::channel remote_channel;
  zx_status_t status = zx::channel::create(0u, &local_channel, &remote_channel);
  if (status != ZX_OK) {
    FX_LOGS(ERROR) << "Bad status creating channel";
    return ZX_ERR_BAD_STATE;
  }
  fidl::InterfaceRequest<audio_fidl::RingBuffer> request = {};
  request.set_channel(std::move(remote_channel));
  audio_fidl::Format fidl_format = {};
  audio_fidl::PcmFormat pcm = {};
  pcm.number_of_channels = channels;
  pcm.bytes_per_sample = format.bytes_per_frame() / channels;
  pcm.valid_bits_per_sample = pcm.bytes_per_sample * 8;
  pcm.frame_rate = frames_per_second;
  pcm.sample_format = audio_fidl::SampleFormat::PCM_SIGNED; // TODO
//  *f.mutable_pcm_format() = pcm;
  fidl_format.set_pcm_format(std::move(pcm));

  if (!stream_config_intf_.is_bound()) {
    FX_LOGS(ERROR) << "Stream channel lost";
    return ZX_ERR_INTERNAL;
  }

  stream_config_intf_->CreateRingBuffer(std::move(fidl_format), std::move(request));
  SetupCommandTimeout();

  ring_buffer_intf_ =
      fidl::InterfaceHandle<audio_fidl::RingBuffer>(std::move(local_channel)).Bind();
  if (!ring_buffer_intf_.is_bound()) {
    FX_LOGS(ERROR) << "Failed to get stream channel";
    return ZX_ERR_INTERNAL;
  }
  ring_buffer_intf_.set_error_handler([](zx_status_t status) -> void {
    FX_PLOGS(ERROR, status) << "AudioDriver failed with error: " << status;
  });

  ring_buffer_intf_->GetProperties([this](audio_fidl::RingBufferProperties props) {
    OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());
    fifo_depth_ = props.fifo_depth();
    AUD_VLOG(TRACE) << "Received fifo depth " << fifo_depth_;
    clock_domain_ = props.clock_domain();
    AUD_VLOG(TRACE) << "Received clock domain " << clock_domain_;
  });

  // Change state, setup our command timeout and we are finished.
  state_ = State::Configuring_SettingFormat;
  configuration_deadline_ = async::Now(owner_->mix_domain().dispatcher()) + kDefaultLongCmdTimeout;
  SetupCommandTimeout();

  auto bytes_per_frame = format.bytes_per_frame();
  // TODO (andresoportuis) wait for fifo to arrive.
  uint32_t fifo_depth_bytes = fifo_depth_;
  fifo_depth_frames_ = (fifo_depth_bytes + bytes_per_frame - 1) / bytes_per_frame;
  fifo_depth_duration_ =
      zx::nsec(TimelineRate::Scale(fifo_depth_frames_, ZX_SEC(1), frames_per_second));

  AUD_VLOG(TRACE) << "Received fifo depth response (in frames) of " << fifo_depth_frames_;

  // Figure out how many frames we need in our ring buffer.
  int64_t min_frames_64 = TimelineRate::Scale(min_ring_buffer_duration_.to_nsecs(),
                                              bytes_per_frame * frames_per_second, ZX_SEC(1));
  int64_t overhead = static_cast<int64_t>(fifo_depth_bytes) + bytes_per_frame - 1;
  bool overflow = ((min_frames_64 == TimelineRate::kOverflow) ||
                   (min_frames_64 > (std::numeric_limits<int64_t>::max() - overhead)));

  if (!overflow) {
    min_frames_64 += overhead;
    min_frames_64 /= bytes_per_frame;
    overflow = min_frames_64 > std::numeric_limits<uint32_t>::max();
  }

  if (overflow) {
    FX_LOGS(ERROR) << "Overflow while attempting to compute ring buffer size in frames.";
    FX_LOGS(ERROR) << "duration        : " << min_ring_buffer_duration_.get();
    FX_LOGS(ERROR) << "bytes per frame : " << bytes_per_frame;
    FX_LOGS(ERROR) << "frames per sec  : " << frames_per_second;
    FX_LOGS(ERROR) << "fifo depth      : " << fifo_depth_bytes;
    return ZX_ERR_INTERNAL;
  }

  AUD_VLOG_OBJ(TRACE, this) << "for audio " << (owner_->is_input() ? "input" : "output")
                            << " -- fifo_depth_bytes:" << fifo_depth_bytes
                            << ", fifo_depth_frames:" << fifo_depth_frames_
                            << ", bytes_per_frame:" << bytes_per_frame;

  zx::vmo ring_buffer_vmo = {};
  uint32_t actual_num_frames = 0;
  ring_buffer_intf_->GetVmo(static_cast<uint32_t>(min_frames_64),
                            kEnablePositionNotifications ? 2 : 0,
                            [&actual_num_frames, &ring_buffer_vmo](audio_fidl::RingBuffer_GetVmo_Result result) {
                              actual_num_frames = result.response().num_frames;
                              ring_buffer_vmo = std::move(result.response().ring_buffer);
                            });


  // We are now Configured. Let our owner know about this important milestone.
  state_ = State::Configured;
  configuration_deadline_ = zx::time::infinite();
  SetupCommandTimeout();
  owner_->OnDriverConfigComplete();
  return ZX_OK;
}

zx_status_t AudioDriver::Start() {
  TRACE_DURATION("audio", "AudioDriver::Start");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());

  // In order to start, we must be in the Configured state.
  //
  // Note: Attempting to start while already started is considered an error because (since we are
  // already started) we will never deliver the OnDriverStartComplete callback. It would be
  // confusing to call it directly from here -- before the user's call to Start even returned.
  if (state_ != State::Configured) {
    FX_LOGS(ERROR) << "Bad state while attempting start (state = " << static_cast<uint32_t>(state_)
                   << ")";
    return ZX_ERR_BAD_STATE;
  }


  return ZX_OK;
}

zx_status_t AudioDriver::Stop() {
  TRACE_DURATION("audio", "AudioDriver::Stop");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());

  // In order to stop, we must be in the Started state.
  // TODO(MTWN-388): make Stop idempotent. Allow Stop when Configured/Stopping; disallow if
  // Shutdown; consider what to do if Uninitialized/MissingDriverInfo/Unconfigured/Configuring. Most
  // importantly, if driver is Starting, queue the request until Start completes (as we cannot
  // cancel driver commands). Finally, handle multiple Stop calls to be in-flight concurrently.
  if (state_ != State::Started) {
    FX_LOGS(ERROR) << "Bad state while attempting stop (state = " << static_cast<uint32_t>(state_)
                   << ")";
    return ZX_ERR_BAD_STATE;
  }

  // Invalidate our timeline transformation here. To outside observers, we are now stopped.
  clock_mono_to_fractional_frame_->Update(TimelineFunction());


  // We were recently in steady state, so assert that we have no configuration timeout at this time.
  FX_DCHECK(configuration_deadline_ == zx::time::infinite());

  return ZX_OK;
}

zx_status_t AudioDriver::SetPlugDetectEnabled(bool enabled) {
  TRACE_DURATION("audio", "AudioDriver::SetPlugDetectEnabled");
  // TODO(MTWN-385): Figure out a better way to assert this!
  OBTAIN_EXECUTION_DOMAIN_TOKEN(token, &owner_->mix_domain());

  if (enabled == pd_enabled_) {
    return ZX_OK;
  }

  audio_stream_cmd_plug_detect_req_t req;
  if (enabled) {
    req.hdr.cmd = AUDIO_STREAM_CMD_PLUG_DETECT;
    req.flags = AUDIO_PDF_ENABLE_NOTIFICATIONS;
    pd_enable_deadline_ = async::Now(owner_->mix_domain().dispatcher()) + kDefaultShortCmdTimeout;
  } else {
    req.hdr.cmd = static_cast<audio_cmd_t>(AUDIO_STREAM_CMD_PLUG_DETECT | AUDIO_FLAG_NO_ACK);
    req.flags = AUDIO_PDF_DISABLE_NOTIFICATIONS;
    pd_enable_deadline_ = zx::time::infinite();
  }
  req.hdr.transaction_id = TXID;


  pd_enabled_ = enabled;
  SetupCommandTimeout();

  return ZX_OK;
}

zx_status_t AudioDriver::ReadMessage(const zx::channel& channel, void* buf, uint32_t buf_size,
                                     uint32_t* bytes_read_out, zx::handle* handle_out) {
  TRACE_DURATION("audio", "AudioDriver::ReadMessage");
  FX_DCHECK(buf != nullptr);
  FX_DCHECK(bytes_read_out != nullptr);
  FX_DCHECK(handle_out != nullptr);
  FX_DCHECK(buf_size >= sizeof(audio_cmd_hdr_t));

  if (!operational()) {
    return ZX_ERR_BAD_STATE;
  }

  zx_status_t res;
  res = channel.read(0, buf, handle_out ? handle_out->reset_and_get_address() : nullptr, buf_size,
                     handle_out ? 1 : 0, bytes_read_out, nullptr);
  if (res != ZX_OK) {
    ShutdownSelf("Error attempting to read channel response", res);
    return res;
  }

  if (*bytes_read_out < sizeof(audio_cmd_hdr_t)) {
    FX_LOGS(ERROR) << "Channel response is too small to hold even a "
                   << "message header (" << *bytes_read_out << " < " << sizeof(audio_cmd_hdr_t)
                   << ").";
    ShutdownSelf("Channel response too small", ZX_ERR_INVALID_ARGS);
    return ZX_ERR_INVALID_ARGS;
  }

  return ZX_OK;
}

zx_status_t AudioDriver::ProcessStartResponse(const audio_rb_cmd_start_resp_t& resp) {
  if (state_ != State::Starting) {
    FX_LOGS(ERROR) << "Received unexpected start response while in state "
                   << static_cast<uint32_t>(state_);
    return ZX_ERR_BAD_STATE;
  }

  if (resp.result != ZX_OK) {
    ShutdownSelf("Error when starting ring buffer", resp.result);
    return resp.result;
  }

  auto format = GetFormat();

  // We are almost Started, so compute the translation from clock-monotonic to ring-buffer-position
  // (in bytes), then update the ring buffer state's transformation and bump the generation counter.
  TimelineFunction func(0, resp.start_time,
                        FractionalFrames<int64_t>(format->frames_per_second()).raw_value(),
                        ZX_SEC(1));
  clock_mono_to_fractional_frame_->Update(func);

  // We are now Started. Let our owner know about this important milestone.
  state_ = State::Started;
  configuration_deadline_ = zx::time::infinite();
  SetupCommandTimeout();
  owner_->OnDriverStartComplete();
  return ZX_OK;
}

zx_status_t AudioDriver::ProcessStopResponse(const audio_rb_cmd_stop_resp_t& resp) {
  TRACE_DURATION("audio", "AudioDriver::ProcessStopResponse");
  if (state_ != State::Stopping) {
    FX_LOGS(ERROR) << "Received unexpected stop response while in state "
                   << static_cast<uint32_t>(state_);
    return ZX_ERR_BAD_STATE;
  }

  if (resp.result != ZX_OK) {
    ShutdownSelf("Error when stopping ring buffer", resp.result);
    return resp.result;
  }

  // We are now stopped and in Configured state. Let our owner know about this important milestone.
  state_ = State::Configured;
  configuration_deadline_ = zx::time::infinite();
  SetupCommandTimeout();
  owner_->OnDriverStopComplete();
  return ZX_OK;
}

// Currently we ignore driver-reported position, using the system-internal clock instead. This is
// benign and can be safely ignored. However, we did not request it, so this may indicate some other
// problem in the driver state machine. Issue a (debug-only) warning, eat the msg, and continue.
zx_status_t AudioDriver::ProcessPositionNotify(const audio_rb_position_notify_t& notify) {
  TRACE_DURATION("audio", "AudioDriver::ProcessPositionNotify");
  if constexpr (kLogPositionNotifications) {
    if ((kPositionNotificationInfoInterval > 0) &&
        (position_notification_count_ % kPositionNotificationInfoInterval == 0)) {
      AUD_LOG_OBJ(INFO, this) << (kEnablePositionNotifications ? "Notification"
                                                               : "Unsolicited notification")
                              << " (1/" << kPositionNotificationInfoInterval
                              << ") Time:" << notify.monotonic_time << ", Pos:" << std::setw(6)
                              << notify.ring_buffer_pos;
    } else if ((kPositionNotificationTraceInterval > 0) &&
               (position_notification_count_ % kPositionNotificationTraceInterval == 0)) {
      AUD_VLOG_OBJ(TRACE, this) << (kEnablePositionNotifications ? "Notification"
                                                                 : "Unsolicited notification")
                                << " (1/" << kPositionNotificationTraceInterval
                                << ") Time:" << notify.monotonic_time << ",  Pos:" << std::setw(6)
                                << notify.ring_buffer_pos;
    } else if ((kPositionNotificationSpewInterval > 0) &&
               (position_notification_count_ % kPositionNotificationSpewInterval == 0)) {
      AUD_VLOG_OBJ(SPEW, this) << (kEnablePositionNotifications ? "Notification"
                                                                : "Unsolicited notification")
                               << " (1/" << kPositionNotificationSpewInterval
                               << ") Time:" << notify.monotonic_time << ", Pos:" << std::setw(6)
                               << notify.ring_buffer_pos;
    }
  }
  // Even if we don't log them, keep a running count of position notifications since START.
  ++position_notification_count_;

  return ZX_OK;
}

void AudioDriver::ShutdownSelf(const char* reason, zx_status_t status) {
  TRACE_DURATION("audio", "AudioDriver::ShutdownSelf");
  if (state_ == State::Shutdown) {
    return;
  }

  if (reason != nullptr) {
    FX_LOGS(INFO) << (owner_->is_input() ? " Input" : "Output") << " shutting down '" << reason
                  << "', status:" << status;
  }

  // Our owner will call our Cleanup function within this call.
  owner_->ShutdownSelf();
  state_ = State::Shutdown;
}

void AudioDriver::SetupCommandTimeout() {
  TRACE_DURATION("audio", "AudioDriver::SetupCommandTimeout");

  // If we have received a late response, report it now.
  if (driver_last_timeout_ != zx::time::infinite()) {
    auto delay = async::Now(owner_->mix_domain().dispatcher()) - driver_last_timeout_;
    driver_last_timeout_ = zx::time::infinite();
    FX_DCHECK(timeout_handler_);
    timeout_handler_(delay);
  }

  zx::time deadline;

  deadline = fetch_driver_info_deadline_;
  deadline = std::min(deadline, configuration_deadline_);
  deadline = std::min(deadline, pd_enable_deadline_);

  if (cmd_timeout_.last_deadline() != deadline) {
    if (deadline != zx::time::infinite()) {
      cmd_timeout_.PostForTime(owner_->mix_domain().dispatcher(), deadline);
    } else {
      cmd_timeout_.Cancel();
    }
  }
}

void AudioDriver::ReportPlugStateChange(bool plugged, zx::time plug_time) {
  TRACE_DURATION("audio", "AudioDriver::ReportPlugStateChange");
  {
    std::lock_guard<std::mutex> lock(plugged_lock_);
    plugged_ = plugged;
    plug_time_ = plug_time;
  }

  if (pd_enabled_) {
    owner_->OnDriverPlugStateChange(plugged, plug_time);
  }
}

zx_status_t AudioDriver::OnDriverInfoFetched(uint32_t info) {
  TRACE_DURATION("audio", "AudioDriver::OnDriverInfoFetched");
  // We should never fetch the same info twice.
  if (fetched_driver_info_ & info) {
    ShutdownSelf("Duplicate driver info fetch\n", ZX_ERR_BAD_STATE);
    return ZX_ERR_BAD_STATE;
  }

  // Record the new piece of info we just fetched.
  FX_DCHECK(state_ == State::MissingDriverInfo);
  fetched_driver_info_ |= info;

  // Have we finished fetching our initial driver info? If so, cancel the timeout, transition to
  // Unconfigured state, and let our owner know that we have finished.
  if ((fetched_driver_info_ & kDriverInfoHasAll) == kDriverInfoHasAll) {
    // We are done. Clear the fetch driver info timeout and let our owner know.
    fetch_driver_info_deadline_ = zx::time::infinite();
    state_ = State::Unconfigured;
    SetupCommandTimeout();
    owner_->OnDriverInfoFetched();
  }

  return ZX_OK;
}

zx_status_t AudioDriver::SendSetGain(const AudioDeviceSettings::GainState& gain_state,
                          audio_set_gain_flags_t set_flags) {
  TRACE_DURATION("audio", "AudioDriver::SendSetGain");

  // We ignore set_flags since the FIDL API requires updates all field of audio_fidl::GainState.
  audio_fidl::GainState gain_state2 = {};
  if (gain_state.muted) {
    *gain_state2.mutable_muted() = true;
  }
  if (gain_state.agc_enabled) {
    *gain_state2.mutable_agc_enabled() = true;
  }
  *gain_state2.mutable_gain_db() = gain_state.gain_db;
  stream_config_intf_->SetGain(std::move(gain_state2));
  return ZX_OK;
}

void AudioDriver::DriverCommandTimedOut() {
  FX_LOGS(WARNING) << "Unexpected driver timeout";
  driver_last_timeout_ = async::Now(owner_->mix_domain().dispatcher());
}

}  // namespace media::audio
