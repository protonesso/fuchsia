// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be found in the LICENSE file.

#include "src/media/audio/audio_core/audio_capturer.h"

#include <lib/fzl/vmo-mapper.h>
#include <lib/zx/vmo.h>

#include <gtest/gtest.h>

#include "src/media/audio/audio_core/audio_admin.h"
#include "src/media/audio/audio_core/audio_device_manager.h"
#include "src/media/audio/audio_core/audio_driver.h"
#include "src/media/audio/audio_core/audio_input.h"
#include "src/media/audio/audio_core/stream_volume_manager.h"
#include "src/media/audio/audio_core/testing/fake_audio_driver.h"
#include "src/media/audio/audio_core/testing/threading_model_fixture.h"
#include "src/media/audio/lib/logging/logging.h"

namespace media::audio {
namespace {

constexpr uint32_t kAudioCapturerUnittestFrameRate = 48000;
constexpr size_t kAudioCapturerUnittestVmarSize = 16ull * 1024;

class AudioCapturerTest : public testing::ThreadingModelFixture {
 public:
  AudioCapturerTest() {
    FX_CHECK(vmo_mapper_.CreateAndMap(kAudioCapturerUnittestVmarSize,
                                      /*flags=*/0, nullptr, &vmo_) == ZX_OK);
  }

 protected:
  void SetUp() override {
    Logging::Init(-media::audio::SPEW, {"audio_capturer_impl_test"});
    testing::ThreadingModelFixture::SetUp();

    auto format = Format::Create(stream_type_).take_value();
    fuchsia::media::InputAudioCapturerConfiguration input_configuration;
    input_configuration.set_usage(fuchsia::media::AudioCaptureUsage::BACKGROUND);
    auto capturer = std::unique_ptr<AudioCapturer>(new AudioCapturer(
        fuchsia::media::AudioCapturerConfiguration::WithInput(std::move(input_configuration)),
        {format}, fidl_capturer_.NewRequest(), &context()));
    capturer_ = capturer.get();
    EXPECT_NE(capturer_, nullptr);

    context().route_graph().AddCapturer(std::move(capturer));
  }

  void TearDown() override {
    // Dropping the channel queues up a reference to the Capturer through its error handler, which
    // will not work since the rest of this class is destructed before the loop and its
    // queued functions are. Here, we ensure the error handler runs before this class' destructors
    // run.
    { auto r = std::move(fidl_capturer_); }
    RunLoopUntilIdle();

    testing::ThreadingModelFixture::TearDown();
  }

 protected:
  AudioCapturer* capturer_;
  fuchsia::media::AudioCapturerPtr fidl_capturer_;

  fzl::VmoMapper vmo_mapper_;
  zx::vmo vmo_;

  fuchsia::media::AudioStreamType stream_type_ = {
      .sample_format = fuchsia::media::AudioSampleFormat::FLOAT,
      .channels = 1,
      .frames_per_second = kAudioCapturerUnittestFrameRate,
  };
};

TEST_F(AudioCapturerTest, CanShutdownWithUnusedBuffer) {
  zx::vmo duplicate;
  ASSERT_EQ(
      vmo_.duplicate(ZX_RIGHT_TRANSFER | ZX_RIGHT_WRITE | ZX_RIGHT_READ | ZX_RIGHT_MAP, &duplicate),
      ZX_OK);
  fidl_capturer_->AddPayloadBuffer(0, std::move(duplicate));
  RunLoopUntilIdle();
}

TEST_F(AudioCapturerTest, RegistersWithRouteGraphIfHasUsageStreamTypeAndBuffers) {
  EXPECT_EQ(context().link_matrix().SourceLinkCount(*capturer_), 0u);

  zx::vmo duplicate;
  ASSERT_EQ(
      vmo_.duplicate(ZX_RIGHT_TRANSFER | ZX_RIGHT_WRITE | ZX_RIGHT_READ | ZX_RIGHT_MAP, &duplicate),
      ZX_OK);

  zx::channel c1, c2;
  ASSERT_EQ(ZX_OK, zx::channel::create(0, &c1, &c2));
  auto input = AudioInput::Create(zx::channel(), &threading_model(), &context().device_manager(),
                                  &context().link_matrix());
  auto fake_driver =
      testing::FakeAudioDriver(std::move(c1), threading_model().FidlDomain().dispatcher());

  auto vmo = fake_driver.CreateRingBufferFakeDriver(PAGE_SIZE);

  input->driver()->Init(std::move(c2));
  fake_driver.StartFakeDriver();
  input->driver()->GetDriverInfo();
  RunLoopUntilIdle();

  input->driver()->Start();

  audio_fidl::PcmSupportedFormats formats = {};
  formats.number_of_channels.push_back(1);
  formats.sample_formats.push_back(audio_fidl::SampleFormat::PCM_SIGNED);
  formats.bytes_per_sample.push_back(2);
  formats.valid_bits_per_sample.push_back(16);
  formats.frame_rates.push_back(48'000);
  fake_driver.set_formats(std::move(formats));
  context().route_graph().AddDevice(input.get());
  RunLoopUntilIdle();

  fidl_capturer_->AddPayloadBuffer(0, std::move(duplicate));

  RunLoopUntilIdle();
  EXPECT_EQ(context().link_matrix().SourceLinkCount(*capturer_), 1u);
}

TEST_F(AudioCapturerTest, CanReleasePacketWithoutDroppingConnection) {
  bool channel_dropped = false;
  fidl_capturer_.set_error_handler([&channel_dropped](auto _) { channel_dropped = true; });
  fidl_capturer_->ReleasePacket(fuchsia::media::StreamPacket{});
  RunLoopUntilIdle();

  // The RouteGraph should still own our capturer.
  EXPECT_FALSE(channel_dropped);
}

}  // namespace
}  // namespace media::audio
