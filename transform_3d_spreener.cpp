//===============================================================================//
// Spreen - transform_3d_spreener.cpp
//===============================================================================//
// MIT License
//
// Copyright (c) 2025 KAR Games
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//===============================================================================//

#include "transform_3d_spreener.h"

void Transform3DSpreener::start() {
	finished = false;

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		WARN_PRINT("Target object freed before starting, aborting Spreener.");
		return;
	}
}

bool Transform3DSpreener::step(double &r_delta) {
	if (finished && _get_spreen()->is_finishable()) {
		// If other springs haven't stabilizied
		return false;
	}

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		_finish();
		return false;
	}

	Variant prop_value = target_instance->get_indexed(property);
	Transform3D transform = prop_value.operator Transform3D();

	Quaternion q = transform.get_basis().get_rotation_quaternion();
	update_spring(q, angular_velocity, goal.get_basis().get_rotation_quaternion(), r_delta);

	Vector3 scale = transform.get_basis().get_scale();
	update_spring(scale, scale_velocity, goal.get_basis().get_scale(), Vector3(0.0f, 0.0f, 0.0f), r_delta);

	Vector3 origin = transform.get_origin();
	update_spring(origin, velocity, goal.get_origin(), Vector3(0.0f, 0.0f, 0.0f), r_delta);

	transform.basis.set_quaternion_scale(q.normalized(), scale);
	transform.origin = origin;

	target_instance->set_indexed(property, transform);

	if (goal.is_equal_approx(transform) && velocity.is_zero_approx() && angular_velocity.is_zero_approx() && scale_velocity.is_zero_approx()) {
		_finish();
		return false;
	}

	return true;
}

Ref<Transform3DSpreener> Transform3DSpreener::update_goal(const Transform3D &p_goal) {
	goal = p_goal;
	return this;
}

Ref<Transform3DSpreener> Transform3DSpreener::set_damping_ratio(real_t p_damping_ratio) {
	damping_ratio = p_damping_ratio;
	return this;
}

Ref<Transform3DSpreener> Transform3DSpreener::set_halflife(real_t p_halflife) {
	halflife = p_halflife;
	return this;
}

void Transform3DSpreener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_goal", "goal"), &Transform3DSpreener::update_goal);
	ClassDB::bind_method(D_METHOD("set_damping_ratio", "damping_ratio"), &Transform3DSpreener::set_damping_ratio);
	ClassDB::bind_method(D_METHOD("set_halflife", "halflife"), &Transform3DSpreener::set_halflife);
}

Transform3DSpreener::Transform3DSpreener(const Object *p_target, const Vector<StringName> &p_property, const Transform3D &p_goal, real_t p_damping_ratio, real_t p_halflife) {
	target = p_target->get_instance_id();
	property = p_property;
	goal = p_goal;
	damping_ratio = p_damping_ratio;
	halflife = p_halflife;
}

Transform3DSpreener::Transform3DSpreener() {
	ERR_FAIL_MSG("Transform3DSpreener can't be created directly. Use the spreen_transform() method in Spreen.");
}
