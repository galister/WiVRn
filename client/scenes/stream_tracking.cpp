/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "application.h"
#include "stream.h"
#include "utils/ranges.h"
#include "wivrn_packets.h"
#include <arpa/inet.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_precision.hpp>
#include <glm/gtx/quaternion.hpp>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <thread>
#include <openxr/openxr.h>

std::string ip_address_to_string(const in_addr & address)
{
	char buffer[128];
	if (inet_ntop(AF_INET, &address, buffer, sizeof(buffer)))
		return buffer;

	return "";
}

std::string ip_address_to_string(const in6_addr & address)
{
	char buffer[128];
	if (inet_ntop(AF_INET6, &address, buffer, sizeof(buffer)))
		return buffer;

	return "";
}
static from_headset::tracking::pose locate_space(device_id device, XrSpace space, XrSpace reference, XrTime time)
{
	XrSpaceVelocity velocity{
	        .type = XR_TYPE_SPACE_VELOCITY,
	};

	XrSpaceLocation location{
	        .type = XR_TYPE_SPACE_LOCATION,
	        .next = &velocity,
	};

	xrLocateSpace(space, reference, time, &location);

	from_headset::tracking::pose res{
	        .device = device,
	        .pose = location.pose,
	        .linear_velocity = velocity.linearVelocity,
	        .angular_velocity = velocity.angularVelocity,
	        .flags = 0,
	};

	if (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
		res.flags |= from_headset::tracking::orientation_valid;

	if (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
		res.flags |= from_headset::tracking::position_valid;

	if (velocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
		res.flags |= from_headset::tracking::linear_velocity_valid;

	if (velocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
		res.flags |= from_headset::tracking::angular_velocity_valid;

	if (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
		res.flags |= from_headset::tracking::orientation_tracked;

	if (location.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
		res.flags |= from_headset::tracking::position_tracked;

	return res;
}

namespace
{
class timer
{
	xr::instance & instance;
	XrTime start = instance.now();
	XrDuration duration = 0;

public:
	timer(xr::instance & instance) :
	        instance(instance) {}
	void pause()
	{
		duration += instance.now() - start;
	}
	void resume()
	{
		start = instance.now();
	}
	XrDuration count()
	{
		pause();
		return duration;
	}
};
} // namespace

static std::optional<std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT>> locate_hands(xr::hand_tracker & hand, XrSpace space, XrTime time)
{
	auto joints = hand.locate(space, time);

	if (joints)
	{
		std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT> poses;
		for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++)
		{
			poses[i] = {
			        .pose = (*joints)[i].first.pose,
			        .linear_velocity = (*joints)[i].second.linearVelocity,
			        .angular_velocity = (*joints)[i].second.angularVelocity,
			        .radius = uint16_t((*joints)[i].first.radius * 10'000),
			};

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::linear_velocity_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::angular_velocity_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_tracked;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_tracked;
		}

		return poses;
	}
	else
		return std::nullopt;
}

XrVector2f eye_to_foveation(XrFovf fov, XrQuaternionf orientation)
{
	glm::quat q = {orientation.w, orientation.x, orientation.y, orientation.z};
	auto euler = glm::eulerAngles(q);

	return XrVector2f{
	        .x = 1 - (euler.y - fov.angleLeft) / (fov.angleRight - fov.angleLeft) * 2,
	        .y = (euler.x - fov.angleDown) / (fov.angleUp - fov.angleDown) * 2 - 1,
	};
}

struct tracking_payload
{
	XrQuaternionf left_eye;
	XrQuaternionf right_eye;
	float face_fb[70];
};

void scenes::stream::tracking()
{
#ifdef __ANDROID__
	// Runtime may use JNI and needs the thread to be attached
	application::instance().setup_jni();
#endif
	std::vector<std::pair<device_id, XrSpace>> spaces = {
	        {device_id::HEAD, application::view()},
	        {device_id::LEFT_AIM, application::left_aim()},
	        {device_id::LEFT_GRIP, application::left_grip()},
	        {device_id::RIGHT_AIM, application::right_aim()},
	        {device_id::RIGHT_GRIP, application::right_grip()},
	};

	XrSpace view_space = application::view();
	XrDuration tracking_period = 1'000'000; // Send tracking data every 1ms
	const XrDuration dt = 100'000;          // Wake up 0.1ms before measuring the position

	XrTime t0 = instance.now();
	from_headset::tracking packet{};

	tracking_payload pload = {
	        .left_eye = {0, 0, 0, 1},
	        .right_eye = {0, 0, 0, 1},
	        .face_fb = {0}};

	struct sockaddr servaddr;
	memset(&servaddr, 0, sizeof(servaddr));
	int sock_fd;
	if (std::holds_alternative<in6_addr>(network_session->address))
	{
		sock_fd = socket(AF_INET6, SOCK_DGRAM, 0);
		auto in6_addr = (sockaddr_in6 *)&servaddr;
		in6_addr->sin6_addr = std::get<1>(network_session->address);
		in6_addr->sin6_family = AF_INET6;
		in6_addr->sin6_port = htons(9009);
		spdlog::info("Tracking address: {}", ip_address_to_string(in6_addr->sin6_addr));
	}
	else
	{
		sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
		auto in_addr = (sockaddr_in *)&servaddr;
		in_addr->sin_addr = std::get<0>(network_session->address);
		in_addr->sin_family = AF_INET;
		in_addr->sin_port = htons(9009);
		spdlog::info("Tracking address: {}", ip_address_to_string(in_addr->sin_addr));
	}

	// TODO ipv6

	while (not exiting)
	{
		try
		{
			XrTime now = instance.now();
			if (now < t0)
				std::this_thread::sleep_for(std::chrono::nanoseconds(t0 - now - dt));

			// If thread can't keep up, skip timestamps
			t0 = std::max(t0, now);

			timer t(instance);

			XrDuration prediction = std::min<XrDuration>(tracking_prediction_offset, 80'000'000);
			// 1 or 2 samples
			for (XrDuration Δt = 0; Δt <= prediction; Δt += std::max<XrDuration>(1, prediction))
			{
				from_headset::hand_tracking hands{};

				packet.production_timestamp = t0;
				hands.production_timestamp = t0;

				packet.timestamp = t0 + Δt;
				hands.timestamp = t0 + Δt;

				try
				{
					auto [flags, views] = session.locate_views(XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, t0 + Δt, view_space);
					assert(views.size() == packet.views.size());

					for (auto [i, j]: utils::zip(views, packet.views))
					{
						j.pose = i.pose;
						j.fov = i.fov;
					}

					packet.flags = flags;

					packet.device_poses.clear();
					std::lock_guard lock(local_floor_mutex);
					for (auto [device, space]: spaces)
					{
						packet.device_poses.push_back(locate_space(device, space, local_floor, t0 + Δt));
					}

					t.pause();
					network_session->send_stream(packet);
					t.resume();

					if (application::get_eye_gaze_supported())
					{
						packet.eye_gaze = locate_space(device_id::EYE_GAZE, application::eye_gaze(), view_space, t0 + Δt);
					}

					if (application::get_hand_tracking_supported())
					{
						hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::left;
						hands.joints = locate_hands(application::get_left_hand(), local_floor, hands.timestamp);
						t.pause();
						network_session->send_stream(hands);
						t.resume();

						hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::right;
						hands.joints = locate_hands(application::get_right_hand(), local_floor, hands.timestamp);
						t.pause();
						network_session->send_stream(hands);
						t.resume();
					}

					{
						auto gazes = application::get_fb_eye_tracker().get_gazes(view_space, t0 + Δt);
						auto expressions = application::get_fb_face_tracker().get_weights(t0 + Δt);
						if (gazes.has_value() && expressions.has_value())
						{
							auto [left, right] = gazes.value();
							pload.left_eye = left.orientation;
							pload.right_eye = right.orientation;

							auto face = expressions.value();
							for (int i = 0; i < 70; i++)
								pload.face_fb[i] = face.weights[i];

							t.pause();
							sendto(sock_fd, &pload, sizeof(tracking_payload), 0, &servaddr, sizeof(servaddr));
							t.resume();
						}
					}

					XrDuration busy_time = t.count();
					// Target: polling between 1 and 5ms, with 20% busy time
					tracking_period = std::clamp<XrDuration>(std::lerp(tracking_period, busy_time * 5, 0.2), 1'000'000, 5'000'000);
				}
				catch (const std::system_error & e)
				{
					if (e.code().category() != xr::error_category() or
					    e.code().value() != XR_ERROR_TIME_INVALID)
						throw;
				}
			}

			t0 += tracking_period;
		}
		catch (std::exception & e)
		{
			spdlog::info("Exception in tracking thread, exiting: {}", e.what());
			exit();
		}
	}
}

void scenes::stream::operator()(to_headset::prediction_offset && packet)
{
	if (packet.offset.count() >= 0)
		tracking_prediction_offset = std::lerp(packet.offset.count(), tracking_prediction_offset.load(), 0.2);
}
