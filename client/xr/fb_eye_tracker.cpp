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

#include "fb_eye_tracker.h"
#include <array>
#include <openxr/openxr.h>

static PFN_xrDestroyEyeTrackerFB xrDestroyEyeTrackerFB{};
XrResult xr::destroy_fb_eye_tracker(XrEyeTrackerFB id)
{
	return xrDestroyEyeTrackerFB(id);
}

xr::fb_eye_tracker::fb_eye_tracker(instance & inst, XrEyeTrackerFB h)
{
	id = h;
	xrGetEyeGazesFB = inst.get_proc<PFN_xrGetEyeGazesFB>("xrGetEyeGazesFB");
	xrDestroyEyeTrackerFB = inst.get_proc<PFN_xrDestroyEyeTrackerFB>("xrDestroyEyeTrackerFB");
}

std::optional<xr::fb_eye_tracker::gazes> xr::fb_eye_tracker::get_gazes(XrSpace space, XrTime time)
{
	if (!id || !xrGetEyeGazesFB)
		return std::nullopt;

	xr::fb_eye_tracker::gazes gazes;

	XrEyeGazesInfoFB info{
	        .type = XR_TYPE_EYE_GAZES_INFO_FB,
	        .next = nullptr,
	        .baseSpace = space,
	        .time = time,
	};

	XrEyeGazesFB eye_gazes{
	        .type = XR_TYPE_EYE_GAZES_FB,
	        .next = nullptr,
	};

	CHECK_XR(xrGetEyeGazesFB(id, &info, &eye_gazes));

	if (eye_gazes.gaze[0].isValid)
	{
		gazes[0].confidence = eye_gazes.gaze[0].gazeConfidence;
		gazes[0].orientation = eye_gazes.gaze[0].gazePose.orientation;
	}

	if (eye_gazes.gaze[1].isValid)
	{
		gazes[1].confidence = eye_gazes.gaze[1].gazeConfidence;
		gazes[1].orientation = eye_gazes.gaze[1].gazePose.orientation;
	}
	else if (eye_gazes.gaze[0].isValid)
	{
		// no left eye, use data from right
		gazes[1].confidence = gazes[0].confidence;
		gazes[1].orientation = gazes[0].orientation;
	}
	else
	{
		// neither eye is valid
		return std::nullopt;
	}

	if (!eye_gazes.gaze[0].isValid)
	{
		// no right eye, use data from left
		gazes[0].confidence = gazes[1].confidence;
		gazes[0].orientation = gazes[1].orientation;
	}

	return gazes;
}
