// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/media/audio/audio_core/test/service/audio_device_service_test.h"

namespace media::audio::test {

const std::string kManufacturer = "Test Manufacturer";
const std::string kProduct = "Test Product";
const audio_stream_unique_id_t kUniqueId = {
    .data = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf},
};
const std::string kUniqueIdString = "000102030405060708090a0b0c0d0e0f";

void AudioDeviceServiceTest::SetUp() {
  HermeticAudioTest::SetUp();

  environment()->ConnectToService(audio_device_enumerator_.NewRequest());
  audio_device_enumerator_.set_error_handler(ErrorHandler());

  zx::channel local_channel;
  zx::channel remote_channel;
  zx_status_t status = zx::channel::create(0u, &local_channel, &remote_channel);
  EXPECT_EQ(ZX_OK, status);

  driver_ = std::make_unique<testing::FakeAudioDriver>(std::move(local_channel), dispatcher());
  ASSERT_NE(driver_, nullptr);
  driver_->set_device_manufacturer(kManufacturer);
  driver_->set_device_product(kProduct);
  driver_->set_stream_unique_id(kUniqueId);
  driver_->StartFakeDriver();

  audio_device_enumerator_.events().OnDeviceAdded = [this](fuchsia::media::AudioDeviceInfo info) {
    devices_.push_back(std::move(info));
  };

  audio_device_enumerator_->AddDeviceByChannel(std::move(remote_channel), "test device", false);
}

void AudioDeviceServiceTest::TearDown() {
  ASSERT_TRUE(audio_device_enumerator_.is_bound());
  audio_device_enumerator_.events().OnDeviceRemoved = [this](uint64_t dev_token) {
    EXPECT_EQ(dev_token, device_token());
    devices_.clear();
  };

  driver_ = nullptr;
  RunLoopUntil([this]() { return devices().empty(); });

  ASSERT_TRUE(audio_device_enumerator_.is_bound());
  audio_device_enumerator_.Unbind();

  HermeticAudioTest::TearDown();
}

// Test that |AddDeviceByChannel| results in an |OnDeviceAdded| event.
TEST_F(AudioDeviceServiceTest, AddDevice) {
  // Expect that the added device is enumerated via the device enumerator.
  RunLoopUntil([this]() { return !devices().empty(); });

  ASSERT_EQ(1u, devices().size());
  auto device = devices()[0];
  EXPECT_EQ(kManufacturer + " " + kProduct, device.name);
  EXPECT_EQ(kUniqueIdString, device.unique_id);
  EXPECT_EQ(false, device.is_input);

  set_device_token(device.token_id);
}

// Test that the info in |GetDevices| matches the info in the |OnDeviceAdded| event.
TEST_F(AudioDeviceServiceTest, GetDevices) {
  RunLoopUntil([this]() { return !devices().empty(); });

  std::optional<std::vector<fuchsia::media::AudioDeviceInfo>> devices;
  audio_device_enumerator().GetDevices(
      [&devices](std::vector<fuchsia::media::AudioDeviceInfo> devices_in) {
        devices = std::move(devices_in);
      });
  RunLoopUntil([&devices]() { return devices.has_value(); });

  ASSERT_EQ(1u, devices->size());
  auto device = (*devices)[0];
  EXPECT_EQ(kManufacturer + " " + kProduct, device.name);
  EXPECT_EQ(kUniqueIdString, device.unique_id);
  EXPECT_EQ(false, device.is_input);

  set_device_token(device.token_id);
}

}  // namespace media::audio::test
