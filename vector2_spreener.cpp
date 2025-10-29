//===============================================================================//
// Spreen - vector2_spreener.cpp
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

#include "vector2_spreener.h"

void Vector2Spreener::start() {
	finished = false;

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		WARN_PRINT("Target object freed before starting, aborting Spreener.");
		return;
	}
}

bool Vector2Spreener::step(double &r_delta) {
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
	Vector2 p = prop_value.operator Vector2();

	update_spring(p, velocity, goal, Vector2(0.0f, 0.0f), r_delta);

	target_instance->set_indexed(property, p);

	if (goal.is_equal_approx(p) && velocity.is_zero_approx()) {
		_finish();
		return false;
	}

	return true;
}

Ref<Vector2Spreener> Vector2Spreener::update_goal(const Vector2 &p_goal) {
	goal = p_goal;
	return this;
}

Ref<Vector2Spreener> Vector2Spreener::set_damping_ratio(real_t p_damping_ratio) {
	damping_ratio = p_damping_ratio;
	return this;
}

Ref<Vector2Spreener> Vector2Spreener::set_halflife(real_t p_halflife) {
	halflife = p_halflife;
	return this;
}

void Vector2Spreener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_goal", "goal"), &Vector2Spreener::update_goal);
	ClassDB::bind_method(D_METHOD("set_damping_ratio", "damping_ratio"), &Vector2Spreener::set_damping_ratio);
	ClassDB::bind_method(D_METHOD("set_halflife", "halflife"), &Vector2Spreener::set_halflife);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping_ratio", PROPERTY_HINT_RANGE, "0,2.0,or_greater"), "set_damping_ratio", "get_damping_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halflife", PROPERTY_HINT_RANGE, "0,2.0,or_greater"), "set_halflife", "get_halflife");
}

Vector2Spreener::Vector2Spreener(const Object *p_target, const Vector<StringName> &p_property, const Vector2 &p_goal, real_t p_damping_ratio, real_t p_halflife) {
	target = p_target->get_instance_id();
	property = p_property;
	goal = p_goal;
	damping_ratio = p_damping_ratio;
	halflife = p_halflife;
	velocity = Vector2(0.0f, 0.0f);
}

Vector2Spreener::Vector2Spreener() {
	ERR_FAIL_MSG("Vector2Spreener can't be created directly. Use the spreen_vector() method in Spreen.");
}
