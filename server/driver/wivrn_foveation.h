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

#pragma once

#include "xrt/xrt_device.h"

#include "wivrn_session.h"

class wivrn_foveation
{
	std::array<xrt_vec2, 2> center_offset = {};
	std::array<from_headset::tracking::view, 2> views = {};
	xrt_space_relation last_gaze = {};

public:
	wivrn_foveation() {}

	void update_tracking(const from_headset::tracking &, const clock_offset &);
	void set_parameters(std::array<to_headset::foveation_parameter, 2> p);
	std::array<xrt_vec2, 2> get_center_uv();
};
