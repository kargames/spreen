//===============================================================================//
// Spreen - transform_2d_spreener.cpp
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

#include "transform_2d_spreener.h"

void Transform2DSpreener::start() {
	finished = false;

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		WARN_PRINT("Target object freed before starting, aborting Spreener.");
		return;
	}
}

bool Transform2DSpreener::step(double &r_delta) {
	Ref<Spreen> spreen = _get_spreen();
	if (finished && (spreen.is_null() || spreen->is_finishable())) {
		// Finished: wait for the other springs to stabilize, or the owning Spreen is gone.
		return false;
	}

	Object *target_instance = ObjectDB::get_instance(target);
	if (!target_instance) {
		_finish();
		return false;
	}

	elapsed_time += r_delta;

	Variant prop_value = spreen_get_indexed(target_instance, property);
	Transform2D transform = prop_value.operator Transform2D();

	real_t rotation = transform.get_rotation();
	real_t goal_rotation = goal.get_rotation();

	// Avoid interpolation issues from angles wrapping over
	real_t working_rotation = 0.0f;
	real_t rotation_difference = Math::angle_difference(rotation, goal_rotation);

	update_spring(working_rotation, angular_velocity, rotation_difference, 0.0f, r_delta);

	rotation += working_rotation;

	Vector2 scale = transform.get_scale();
	update_spring(scale, scale_velocity, goal.get_scale(), Vector2(0.0f, 0.0f), r_delta);

	real_t skew = transform.get_skew();
	real_t goal_skew = goal.get_skew();

	real_t working_skew = 0.0f;
	real_t skew_difference = Math::angle_difference(skew, goal_skew);

	update_spring(working_skew, skew_velocity, skew_difference, 0.0f, r_delta);

	skew += working_skew;

	Vector2 origin = transform.get_origin();
	update_spring(origin, velocity, goal.get_origin(), Vector2(0.0f, 0.0f), r_delta);

	transform = Transform2D(rotation, scale, skew, origin);
	spreen_set_indexed(target_instance, property, transform);

	if (goal.is_equal_approx(transform) && velocity_settled(angular_velocity, 1.0f) && velocity_settled(scale_velocity, goal.get_scale()) && velocity_settled(skew_velocity, 1.0f) && velocity_settled(velocity, goal.get_origin())) {
		_finish();
		return false;
	}

	return true;
}

Ref<Transform2DSpreener> Transform2DSpreener::update_goal(const Transform2D &p_goal) {
	goal = p_goal;
	return this;
}

Ref<Transform2DSpreener> Transform2DSpreener::set_damping_ratio(real_t p_damping_ratio) {
	ERR_FAIL_COND_V_MSG(p_damping_ratio <= 0, this, "Spreener damping_ratio must be greater than 0.");
	damping_ratio = p_damping_ratio;
	return this;
}

Ref<Transform2DSpreener> Transform2DSpreener::set_halflife(real_t p_halflife) {
	ERR_FAIL_COND_V_MSG(p_halflife <= 0, this, "Spreener halflife must be greater than 0.");
	halflife = p_halflife;
	return this;
}

void Transform2DSpreener::_bind_methods() {
	ClassDB::bind_method(D_METHOD("update_goal", "goal"), &Transform2DSpreener::update_goal);
	ClassDB::bind_method(D_METHOD("set_damping_ratio", "damping_ratio"), &Transform2DSpreener::set_damping_ratio);
	ClassDB::bind_method(D_METHOD("set_halflife", "halflife"), &Transform2DSpreener::set_halflife);
}

Transform2DSpreener::Transform2DSpreener(const Object *p_target, const NodePath &p_property, const Transform2D &p_goal, real_t p_damping_ratio, real_t p_halflife) {
	target = p_target->get_instance_id();
	property = p_property;
	goal = p_goal;
	damping_ratio = p_damping_ratio;
	halflife = p_halflife;
	velocity = Vector2(0.0f, 0.0f);
	scale_velocity = Vector2(0.0f, 0.0f);
	angular_velocity = 0.0f;
	skew_velocity = 0.0f;
}

Transform2DSpreener::Transform2DSpreener() {
	ERR_FAIL_MSG("Transform2DSpreener can't be created directly. Use the spreen_transform() method in Spreen.");
}
