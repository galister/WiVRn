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
#include "math/m_api.h"
#include "math/m_space.h"
#include "util/u_logging.h"
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
		return {scale * std::atan2(-q.z, q.w), pitch};
	}

	return {
	        std::atan2(2.0f * (q.x * q.z + q.w * q.y),
	                   q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
	        pitch};
}

std::array<xrt_vec2, 2> wivrn_foveation::get_center_uv()
{
	std::array<xrt_vec2, 2> uvs;
	auto e = yaw_pitch(last_gaze);

	for (int i = 0; i < 2; i++)
	{
		uvs[i].x = (e.x - views[i].fov.angleLeft) / (views[i].fov.angleRight - views[i].fov.angleLeft) * 2 - 1 + center_offset[i].x;
		uvs[i].y = (e.y - views[i].fov.angleDown) / (views[i].fov.angleUp - views[i].fov.angleDown) * 2 - 1 + center_offset[i].y;
	}

	return uvs;
}

void wivrn_foveation::update_tracking(const from_headset::tracking & tracking, const clock_offset & offset)
{
	const uint8_t orientation_ok = from_headset::tracking::orientation_valid | from_headset::tracking::orientation_tracked;

	views = tracking.views;

	std::optional<xrt_quat> head;

	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device_id::HEAD)
			continue;

		if ((pose.flags & orientation_ok) != orientation_ok)
			return;

		head = xrt_cast(pose.pose.orientation);
		break;
	}

	if (!head.has_value())
		return;

	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device_id::EYE_GAZE)
			continue;

		if ((pose.flags & orientation_ok) != orientation_ok)
			return;

		xrt_quat qgaze = xrt_cast(pose.pose.orientation);
		math_quat_unrotate(&qgaze, &head.value(), &last_gaze);
		break;
	}
}

void wivrn_foveation::set_parameters(std::array<to_headset::foveation_parameter, 2> p)
{
	center_offset[0] = {p[0].x.center, p[0].y.center};
	center_offset[1] = {p[1].x.center, p[1].y.center};
}
