
/*
 * WiVRn VR streaming
 * Copyright (C) 2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2024  Patrick Nicolas <patricknicolas@laposte.net>
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

#include "../common/wivrn_packets.h"
#include "utils/handle.h"
#include <array>
#include <optional>
#include <utility>
#include <openxr/openxr.h>

#include "instance.h"

namespace xr
{
XrResult destroy_fb_eye_tracker(XrEyeTrackerFB);

struct EyeGaze
{
	XrQuaternionf orientation;
	float confidence;
};

class fb_eye_tracker : public utils::handle<XrEyeTrackerFB, destroy_fb_eye_tracker>
{
	PFN_xrGetEyeGazesFB xrGetEyeGazesFB{};
	PFN_xrDestroyEyeTrackerFB xrDestroyEyeTrackerFB{};

public:
	fb_eye_tracker() = default;
	fb_eye_tracker(instance & inst, XrEyeTrackerFB h);
	fb_eye_tracker(fb_eye_tracker &&) = default;
	fb_eye_tracker & operator=(fb_eye_tracker &&) = default;

	~fb_eye_tracker()
	{
		if (id != XR_NULL_HANDLE && xrDestroyEyeTrackerFB)
			xrDestroyEyeTrackerFB(id);
	}

	using gazes = std::array<EyeGaze, 2>;

	std::optional<gazes> get_gazes(XrSpace space, XrTime time);
};
} // namespace xr
