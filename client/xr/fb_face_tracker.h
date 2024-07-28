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
XrResult destroy_fb_face_tracker(XrFaceTracker2FB);

struct FbExpressionWeights
{
	std::array<float, XR_FACE_EXPRESSION2_COUNT_FB> weights;
	std::array<float, XR_FACE_CONFIDENCE2_COUNT_FB> confidences;
};

class fb_face_tracker : public utils::handle<XrFaceTracker2FB, destroy_fb_face_tracker>
{
	PFN_xrGetFaceExpressionWeights2FB xrGetFaceExpressionWeights2FB{};
	PFN_xrDestroyFaceTracker2FB xrDestroyFaceTracker2FB{};

public:
	fb_face_tracker() = default;
	fb_face_tracker(instance & inst, XrFaceTracker2FB h);
	fb_face_tracker(fb_face_tracker &&) = default;
	fb_face_tracker & operator=(fb_face_tracker &&) = default;

	~fb_face_tracker()
	{
		if (id != XR_NULL_HANDLE && xrDestroyFaceTracker2FB)
			xrDestroyFaceTracker2FB(id);
	}

	std::optional<FbExpressionWeights> get_weights(XrTime time);
};
} // namespace xr
