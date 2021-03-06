// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_window.h"

#include <stdio.h>

#include "tests/common/utils.h"
#include "tests/common/vk_app_state.h"
#include "tests/common/vk_surface.h"
#include "tests/common/vk_swapchain.h"
#include "tests/common/vk_swapchain_queue.h"
#include "vulkan_device.h"

bool
VulkanWindow::init(VulkanDevice * device, const VulkanWindow::Config & config)
{
  device_ = device;

  // Allocate display surface, and determine whether it's possible to directly
  // render to the swapchain with it.
  VkSurfaceKHR window_surface =
    vk_app_state_create_surface(&device->vk_app_state(), config.window_width, config.window_height);

  VkImageUsageFlags image_usage = 0;

  // Check that rendering directly to the swapchain is supported
  if (config.require_swapchain_image_shader_storage)
    {
      vk_device_surface_info_t surface_info;
      vk_device_surface_info_init(&surface_info,
                                  device_->vk_physical_device(),
                                  window_surface,
                                  device_->vk_instance());

      VkFormat format = vk_device_surface_info_find_presentation_format(&surface_info,
                                                                        VK_IMAGE_USAGE_STORAGE_BIT,
                                                                        VK_FORMAT_UNDEFINED);
      vk_device_surface_info_destroy(&surface_info);

      if (format == VK_FORMAT_UNDEFINED)
        {
          fprintf(stderr, "ERROR: display surface does not support VK_IMAGE_USAGE_STORAGE_BIT!\n");
          return false;
        }

      image_usage = VK_IMAGE_USAGE_STORAGE_BIT;
    }

  VkFormat wanted_format = config.wanted_format;

  const vk_swapchain_config_t swapchain_config = {
    .instance        = device_->vk_instance(),
    .device          = device_->vk_device(),
    .physical_device = device_->vk_physical_device(),
    .allocator       = device_->vk_allocator(),

    .present_queue_family  = device_->graphics_queue_family(),
    .present_queue_index   = 0,
    .graphics_queue_family = device_->graphics_queue_family(),
    .graphics_queue_index  = 0,

    .surface_khr = window_surface,

    .max_frames              = 3,
    .pixel_format            = wanted_format,
    .disable_vsync           = config.disable_vsync,
    .image_usage_flags       = image_usage,
    .use_presentation_layout = true,
  };
  swapchain_ = vk_swapchain_create(&swapchain_config);

  // Sanity check.
  VkSurfaceFormatKHR surface_format = vk_swapchain_get_format(swapchain_);
  if (wanted_format != VK_FORMAT_UNDEFINED && surface_format.format != wanted_format)
    {
      fprintf(stderr, "WARNING: Could not find wanted pixel format, colors may be wrong!\n");
    }

  if (config.verbose)
    vk_swapchain_print(swapchain_);

  info_.image_count    = vk_swapchain_get_image_count(swapchain_);
  info_.extent         = vk_swapchain_get_extent(swapchain_);
  info_.surface        = window_surface;
  info_.surface_format = surface_format;

  if (config.enable_swapchain_queue)
    {
      vk_swapchain_queue_config_t queue_config = {
        .swapchain    = swapchain_,
        .queue_family = device_->graphics_queue_family(),
        .queue_index  = 0u,
        .device       = device_->vk_device(),
        .allocator    = device_->vk_allocator(),

        .enable_framebuffers   = config.enable_framebuffers,
        .sync_semaphores_count = config.sync_semaphores_count,
      };
      swapchain_queue_ = vk_swapchain_queue_create(&queue_config);
    }

  return true;
}

VulkanWindow::~VulkanWindow()
{
  if (swapchain_queue_)
    vk_swapchain_queue_destroy(swapchain_queue_);

  if (swapchain_)
    vk_swapchain_destroy(swapchain_);
}

bool
VulkanWindow::acquireSwapchainImage()
{
  ASSERT_MSG(!swapchain_queue_, "Calling this method requires enable_swapchain_queue=false");
  return vk_swapchain_acquire_next_image(swapchain_, &image_index_);
}

void
VulkanWindow::presentSwapchainImage()
{
  ASSERT_MSG(!swapchain_queue_, "Calling this method requires enable_swapchain_queue=false");
  vk_swapchain_present_image(swapchain_);
}

bool
VulkanWindow::acquireSwapchainQueueImage()
{
  ASSERT_MSG(swapchain_queue_, "Calling this method requires enable_swapchain_queue=true");

  swapchain_queue_image_ = vk_swapchain_queue_acquire_next_image(swapchain_queue_);
  if (!swapchain_queue_image_)
    return false;

  image_index_ = vk_swapchain_queue_get_index(swapchain_queue_);
  return true;
}

void
VulkanWindow::presentSwapchainQueueImage()
{
  ASSERT_MSG(swapchain_queue_, "Calling this method requires enable_swapchain_queue=true");
  vk_swapchain_queue_submit_and_present_image(swapchain_queue_);
}

bool
VulkanWindow::handleUserEvents()
{
  return vk_app_state_poll_events(&device_->vk_app_state());
}

void
VulkanWindow::waitIdle()
{
  vkDeviceWaitIdle(device_->vk_device());
}
