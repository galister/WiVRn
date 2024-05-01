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

#include "fb2_face_tracker.h"
#include <openxr/openxr.h>

std::optional<xr::fb2_face_tracker::expressions> xr::fb2_face_tracker::get_weights(XrTime time)
{
	if (!id || !xrGetFaceExpressionWeights2FB)
		return std::nullopt;

	xr::fb2_face_tracker::expressions expressions;

	XrFaceExpressionInfo2FB info{
	        .type = XR_TYPE_FACE_EXPRESSION_INFO2_FB,
	        .next = nullptr,
	        .time = time,
	};

	XrFaceExpressionWeights2FB expression_weights{
	        .type = XR_TYPE_FACE_EXPRESSION_WEIGHTS2_FB,
	        .next = nullptr,
	        .weightCount = expressions.weights.size(),
	        .weights = expressions.weights.data(),
	        .confidenceCount = expressions.confidences.size(),
	        .confidences = expressions.confidences.data(),
	};

	CHECK_XR(xrGetFaceExpressionWeights2FB(id, &info, &expression_weights));

	if (!expression_weights.isValid)
		return std::nullopt;

	return expressions;
}
