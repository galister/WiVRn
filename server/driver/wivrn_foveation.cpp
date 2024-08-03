/*
 * WiVRn VR streaming
 * Copyright (C) 2024  galister <galister@librevr.org>
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

#include "wivrn_foveation.h"

#include "driver/xrt_cast.h"
#include "math/m_space.h"
#include "wivrn_packets.h"
#include "xrt/xrt_defines.h"
#include <cmath>

#include <openxr/openxr.h>

xrt_vec2 yaw_pitch(xrt_quat q)
{
	float sine_theta = std::clamp(-2.0f * (q.y * q.z - q.w * q.x), -1.0f, 1.0f);

	float pitch = std::asin(sine_theta);

	if (std::abs(sine_theta) > 0.99999f)
	{
		float scale = std::copysign(2.0, sine_theta);
		return {scale * -std::atan2(-q.z, q.w), pitch};
	}

	return {
	        -std::atan2(2.0f * (q.x * q.z + q.w * q.y),
	                    q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
	        pitch};
}

std::array<xrt_vec2, 2> wivrn_foveation::get_center_uv()
{
	auto e = yaw_pitch(last_gaze.pose.orientation);

	std::array<xrt_vec2, 2> uvs;

	for (int i = 0; i < 2; i++)
	{
		uvs[i].x = (e.x - views[i].fov.angleLeft) / (views[i].fov.angleRight - views[i].fov.angleLeft) * 2 - 1 + center_offset[i].x;
		uvs[i].y = 1 - (e.y - views[i].fov.angleDown) / (views[i].fov.angleUp - views[i].fov.angleDown) * 2 + center_offset[i].y;
	}

	return uvs;
}

static xrt_space_relation_flags cast_flags(uint8_t in_flags)
{
	std::underlying_type_t<xrt_space_relation_flags> flags = 0;
	if (in_flags & from_headset::tracking::position_valid)
		flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;

	if (in_flags & from_headset::tracking::orientation_valid)
		flags |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;

	if (in_flags & from_headset::tracking::linear_velocity_valid)
		flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;

	if (in_flags & from_headset::tracking::angular_velocity_valid)
		flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;

	if (in_flags & from_headset::tracking::position_tracked)
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;

	if (in_flags & from_headset::tracking::orientation_tracked)
		flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	return xrt_space_relation_flags(flags);
}

static xrt_space_relation to_relation(const from_headset::tracking::pose & pose)
{
	return {
	        .relation_flags = cast_flags(pose.flags),
	        .pose = xrt_cast(pose.pose),
	        .linear_velocity = xrt_cast(pose.linear_velocity),
	        .angular_velocity = xrt_cast(pose.angular_velocity),
	};
}

void wivrn_foveation::update_tracking(const from_headset::tracking & tracking, const clock_offset & offset)
{
	const uint8_t orientation_ok = from_headset::tracking::orientation_valid | from_headset::tracking::orientation_tracked;

	views = tracking.views;

	xrt_relation_chain xrc{};

	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device_id::HEAD)
			continue;

		if ((pose.flags & orientation_ok) != orientation_ok)
			return;

		xrt_space_relation xsr = to_relation(pose);
		m_relation_chain_push_inverted_relation(&xrc, &xsr);
		break;
	}

	// there was no HEAD device in the tracking data
	if (xrc.step_count == 0)
		return;

	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device_id::EYE_GAZE)
			continue;

		if ((pose.flags & orientation_ok) != orientation_ok)
			return;

		xrt_space_relation xsr = to_relation(pose);
		m_relation_chain_push_relation(&xrc, &xsr);

		m_relation_chain_resolve(&xrc, &xsr);
		last_gaze = xsr;
	}
}

void wivrn_foveation::set_parameters(std::array<to_headset::foveation_parameter, 2> p)
{
	center_offset[0] = {p[0].x.center, p[0].y.center};
	center_offset[1] = {p[1].x.center, p[1].y.center};
}
