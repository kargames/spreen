//===============================================================================//
// Spreen - basis_spreener.cpp
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

#include "basis_spreener.h"

void BasisSpreener::start() {
	finished = false;

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		WARN_PRINT("Target object freed before starting, aborting Spreener.");
		return;
	}
}

bool BasisSpreener::step(double &r_delta) {
	if (finished && _get_spreen()->is_finishable()) {
		// If other springs haven't stabilizied
		return false;
	}

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		_finish();
		return false;
	}

	elapsed_time += r_delta;

	Variant prop_value = target_instance->get_indexed(property);
	Basis basis = prop_value.operator Basis();

	Quaternion q = basis.get_rotation_quaternion();
	update_spring(q, angular_velocity, goal.get_rotation_quaternion(), r_delta);

	Vector3 scale = basis.get_scale();
	update_spring(scale, scale_velocity, goal.get_scale(), Vector3(0.0f, 0.0f, 0.0f), r_delta);

	basis.set_quaternion_scale(q.normalized(), scale);

	target_instance->set_indexed(property, basis);

	if (goal.is_equal_approx(basis) && angular_velocity.is_zero_approx() && scale_velocity.is_zero_approx()) {
		_finish();
		return false;
	}

	return true;
}

Ref<BasisSpreener> BasisSpreener::update_goal(const Basis &p_goal) {
	goal = p_goal;
	return this;
}

Ref<BasisSpreener> BasisSpreener::set_damping_ratio(real_t p_damping_ratio) {
	damping_ratio = p_damping_ratio;
	return this;
}

Ref<BasisSpreener> BasisSpreener::set_halflife(real_t p_halflife) {
	halflife = p_halflife;
	return this;
}

void BasisSpreener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_goal", "goal"), &BasisSpreener::update_goal);
	ClassDB::bind_method(D_METHOD("set_damping_ratio", "damping_ratio"), &BasisSpreener::set_damping_ratio);
	ClassDB::bind_method(D_METHOD("set_halflife", "halflife"), &BasisSpreener::set_halflife);
}

BasisSpreener::BasisSpreener(const Object *p_target, const Vector<StringName> &p_property, const Basis &p_goal, real_t p_damping_ratio, real_t p_halflife) {
	target = p_target->get_instance_id();
	property = p_property;
	goal = p_goal;
	damping_ratio = p_damping_ratio;
	halflife = p_halflife;
	scale_velocity = Vector3(0.0f, 0.0f, 0.0f);
	angular_velocity = Vector3(0.0f, 0.0f, 0.0f);
}

BasisSpreener::BasisSpreener() {
	ERR_FAIL_MSG("BasisSpreener can't be created directly. Use the spreen_basis() method in Spreen.");
}
