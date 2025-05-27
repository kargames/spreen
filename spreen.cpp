//===============================================================================//
// Spreen - spreen.cpp
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

#include "spreen.h"

#include "scene/main/node.h"
#include "basis_spreener.h"
#include "float_spreener.h"
#include "transform_2d_spreener.h"
#include "transform_3d_spreener.h"
#include "vector2_spreener.h"
#include "vector3_spreener.h"

#define CHECK_VALID()                                                                                      \
	ERR_FAIL_COND_V_MSG(!valid, nullptr, "Spreen invalid. Either finished or created outside spreen tree."); \
	ERR_FAIL_COND_V_MSG(started, nullptr, "Can't append to a Spreen that has started. Use stop() first.");

real_t Spreener::get_damping_ratio() const {
	return damping_ratio;
}

real_t Spreener::get_halflife() const {
	return halflife;
}

void Spreener::set_spreen(const Ref<Spreen> &p_spreen) {
	spreen_id = p_spreen->get_instance_id();
}

Ref<Spreen> Spreener::_get_spreen() {
	return Ref<Spreen>(ObjectDB::get_instance(spreen_id));
}

void Spreener::_finish() {
  finished = true;
  emit_signal(SceneStringName(finished));
}

void Spreener::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_damping_ratio"), &BasisSpreener::get_damping_ratio);
	ClassDB::bind_method(D_METHOD("get_halflife"), &BasisSpreener::get_halflife);

	ADD_SIGNAL(MethodInfo("finished"));

	ADD_PROPERTY_DEFAULT("damping_ratio", 0.5);
	ADD_PROPERTY_DEFAULT("halflife", 0.5);
}

void Spreener::update_spring(real_t &x, real_t &v, const real_t x_goal, real_t v_goal, double delta) {
  real_t g = x_goal;
	real_t q = v_goal;
	real_t d = halflife_to_damping(halflife);
	real_t s = damping_ratio_to_stiffness(damping_ratio, d);
	real_t c = g + (d * q) / (s + CMP_EPSILON);
	real_t y = d / 2.0f;

	if (Math::absf(1. - damping_ratio) < CMP_EPSILON) // Critically Damped
	{
		real_t j0 = x - c;
		real_t j1 = v + j0 * y;

		real_t eydt = Math::exp(-y * delta);

		x = j0 * eydt + delta * j1 * eydt + c;
		v = -y * j0 * eydt - y * delta * j1 * eydt + j1 * eydt;
	}
	else if (damping_ratio < 1.) // Under Damped
	{
		real_t w = Math::sqrt(s - (d * d) / 4.0f);
		real_t j = Math::sqrt(Math::pow(v + y * (x - c), (real_t)2.0f) / (w * w + CMP_EPSILON) + Math::pow(x - c, (real_t)2.0f));
		real_t p = Math::atan((v + (x - c) * y) / (-(x - c) * w + CMP_EPSILON));

		j = (x - c) > 0.0 ? j : -j;

		real_t eydt = Math::exp(-y * delta);

		x = j * eydt * Math::cos(w * delta + p) + c;
		v = -y * j * eydt * Math::cos(w * delta + p) - w * j * eydt * Math::sin(w * delta + p);
	}
	else if (damping_ratio > 1.) // Over Damped
	{
		real_t y0 = (d + Math::sqrt(d * d - 4 * s)) / 2.0f;
		real_t y1 = (d - Math::sqrt(d * d - 4 * s)) / 2.0f;
		real_t j1 = (c * y0 - x * y0 - v) / (y1 - y0);
		real_t j0 = x - j1 - c;

		real_t ey0dt = Math::exp(-y0 * delta);
		real_t ey1dt = Math::exp(-y1 * delta);

		x = j0 * ey0dt + j1 * ey1dt + c;
		v = -y0 * j0 * ey0dt - y1 * j1 * ey1dt;
	}
}

void Spreener::update_spring(Vector2 &p, Vector2 &v, const Vector2 &p_goal, const Vector2 &v_goal, double delta) {
	real_t x = p.x;
  real_t y = p.y;
  real_t dx = v.x; 
  real_t dy = v.y; 

  update_spring(x, dx, p_goal.x, 0.0f, delta);
  update_spring(y, dy, p_goal.y, 0.0f, delta);

  p = Vector2(x, y);
  v = Vector2(dx, dy);
}

void Spreener::update_spring(Vector3 &p, Vector3 &v, const Vector3 &p_goal, const Vector3 &v_goal, double delta) {
	real_t x = p.x;
  real_t y = p.y;
  real_t z = p.z;
  real_t dx = v.x; 
  real_t dy = v.y; 
  real_t dz = v.z; 

  update_spring(x, dx, p_goal.x, 0.0f, delta);
  update_spring(y, dy, p_goal.y, 0.0f, delta);
  update_spring(z, dz, p_goal.z, 0.0f, delta);

  p = Vector3(x, y, z);
  v = Vector3(dx, dy, dz);
}

void Spreener::update_spring(Quaternion &q, Vector3 &v, const Quaternion &q_goal, double delta) {
  // g == c == q_goal, assming v_goal = 0
  Vector3 x_minus_c = quat_to_scaled_angle_axis(quat_abs(q * q_goal.inverse()));
  real_t d = halflife_to_damping(halflife);
	real_t s = damping_ratio_to_stiffness(damping_ratio, d);
	real_t y = d / 2.0;

  if (Math::absf(1. - damping_ratio) < CMP_EPSILON) // Critically Damped
  {
    Vector3 j0 = x_minus_c;
		Vector3 j1 = v + j0 * y;

		real_t eydt = Math::exp(-y * delta);

		q = quat_from_scaled_angle_axis(eydt * (j0 + j1 * delta)) * q_goal;
		v = eydt * (v - j1 * y * delta);
  }
  else if (damping_ratio < 1.) // Under Damped
	{
		real_t w = Math::sqrt(s - (d * d) / 4.0);
		real_t eydt = Math::exp(-y * delta);

		Vector3 j, p, cos_pw, sin_pw;
		for (int i = 0; i < 3; i++)	
		{
			j[i] = Math::sqrt(Math::pow(v[i] + y * x_minus_c[i], (real_t)2.0f) / (w * w + CMP_EPSILON) + Math::pow(x_minus_c[i], (real_t)2.0f));
			p[i] = Math::atan((v[i] + x_minus_c[i] * y) / (-x_minus_c[i] * w + CMP_EPSILON));
			
			j[i] = x_minus_c[i] > 0.0 ? j[i] : -j[i];
			cos_pw[i] = Math::cos(p[i] + w * delta);
			sin_pw[i] = Math::sin(p[i] + w * delta);
		}

		q = quat_from_scaled_angle_axis(j * eydt * cos_pw) * q_goal;
		v = -y * j * eydt * cos_pw - w * j * eydt * sin_pw;
	}
	else if (damping_ratio > 1.) // Over Damped
	{
		real_t y0 = (d + Math::sqrt(d * d - 4 * s)) / 2.0;
		real_t y1 = (d - Math::sqrt(d * d - 4 * s)) / 2.0;
		Vector3 j1 = (-x_minus_c * y0 - v) / (y1 - y0);
		Vector3 j0 = x_minus_c - j1;

		real_t ey0dt = Math::exp(-y0 * delta);
		real_t ey1dt = Math::exp(-y1 * delta);

		q = quat_from_scaled_angle_axis(j0 * ey0dt + j1 * ey1dt) * q_goal;
		v = -y0 * j0 * ey0dt - y1 * j1 * ey1dt;
	}
}

void Spreen::_start_spreeners() {
	if (spreeners.is_empty()) {
		dead = true;
		ERR_FAIL_MSG("Spreen without commands, aborting.");
	}

	for (Ref<Spreener> &spreener : spreeners) {
		spreener->start();
	}
}

void Spreen::_stop_internal(bool p_reset) {
	running = false;
	if (p_reset) {
		started = false;
		dead = false;
		total_time = 0;
	}
}

Ref<FloatSpreener> Spreen::spreen_float(const Object *p_target, const NodePath &p_property, real_t p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
  CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::FLOAT, nullptr, "The spreened property does not match the required type.");

  Ref<FloatSpreener> spreener = memnew(FloatSpreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

Ref<Vector2Spreener> Spreen::spreen_vector2(const Object *p_target, const NodePath &p_property, const Vector2 &p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
	CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::VECTOR2, nullptr, "The spreened property does not match the required type.");

  Ref<Vector2Spreener> spreener = memnew(Vector2Spreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

Ref<Transform2DSpreener> Spreen::spreen_transform_2d(const Object *p_target, const NodePath &p_property, const Transform2D &p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
	CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::TRANSFORM2D, nullptr, "The spreened property does not match the required type.");

  Ref<Transform2DSpreener> spreener = memnew(Transform2DSpreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

Ref<Vector3Spreener> Spreen::spreen_vector3(const Object *p_target, const NodePath &p_property, const Vector3 &p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
	CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::VECTOR3, nullptr, "The spreened property does not match the required type.");

  Ref<Vector3Spreener> spreener = memnew(Vector3Spreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

Ref<BasisSpreener> Spreen::spreen_basis(const Object *p_target, const NodePath &p_property, const Basis &p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
	CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::BASIS, nullptr, "The spreened property does not match the required type.");

  Ref<BasisSpreener> spreener = memnew(BasisSpreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

Ref<Transform3DSpreener> Spreen::spreen_transform_3d(const Object *p_target, const NodePath &p_property, const Transform3D &p_goal, real_t p_damping_ratio, real_t p_halflife) {
  ERR_FAIL_NULL_V(p_target, nullptr);
	CHECK_VALID();

  Vector<StringName> property_subnames = p_property.get_as_property_path().get_subnames();
#ifdef DEBUG_ENABLED
	bool prop_valid;
	const Variant &prop_value = p_target->get_indexed(property_subnames, &prop_valid);
	ERR_FAIL_COND_V_MSG(!prop_valid, nullptr, vformat("The spreened property \"%s\" does not exist in object \"%s\".", p_property, p_target));
#else
	const Variant &prop_value = p_target->get_indexed(property_subnames);
#endif

  ERR_FAIL_COND_V_MSG(prop_value.get_type() != Variant::TRANSFORM3D, nullptr, "The spreened property does not match the required type.");

  Ref<Transform3DSpreener> spreener = memnew(Transform3DSpreener(p_target, property_subnames, p_goal, p_damping_ratio, p_halflife));
  append(spreener);
  return spreener;
}

void Spreen::append(Ref<Spreener> p_spreener) {
  p_spreener->set_spreen(this);
  spreeners.push_back(p_spreener);
}

void Spreen::stop() {
	_stop_internal(true);
}

void Spreen::pause() {
	_stop_internal(false);
}

void Spreen::play() {
	ERR_FAIL_COND_MSG(!valid, "Spreen invalid. Either finished or created outside scene tree.");
	ERR_FAIL_COND_MSG(dead, "Can't play finished Spreen, use stop() first to reset its state.");
	running = true;
}

void Spreen::kill() {
	running = false; // For the sake of is_running().
	dead = true;
}

bool Spreen::is_finishable() {
  return finishable;
}

bool Spreen::is_running() {
	return running;
}

bool Spreen::is_valid() {
	return valid;
}

void Spreen::clear() {
	valid = false;
	spreeners.clear();
}

Ref<Spreen> Spreen::bind_node(const Node *p_node) {
	ERR_FAIL_NULL_V(p_node, this);

	bound_node = p_node->get_instance_id();
	is_bound = true;
	return this;
}

Ref<Spreen> Spreen::set_process_mode(SpreenProcessMode p_mode) {
	process_mode = p_mode;
	return this;
}

Spreen::SpreenProcessMode Spreen::get_process_mode() {
	return process_mode;
}

Ref<Spreen> Spreen::set_pause_mode(SpreenPauseMode p_mode) {
	pause_mode = p_mode;
	return this;
}

Spreen::SpreenPauseMode Spreen::get_pause_mode() {
	return pause_mode;
}

Ref<Spreen> Spreen::set_finishable(bool p_finishable) {
  finishable = p_finishable;
  return this;
}

bool Spreen::custom_step(double p_delta) {
	bool r = running;
	running = true;
	bool ret = step(p_delta);
	running = running && r; // Running might turn false when Tween finished.
	return ret;
}

bool Spreen::step(double p_delta) {
  if (dead) {
		return false;
	}

	if (is_bound) {
		Node *node = get_bound_node();
		if (node) {
			if (!node->is_inside_tree()) {
				return true;
			}
		} else {
			return false;
		}
	}

  if (!running) {
		return true;
	}

  if (!started) {
		if (spreeners.is_empty()) {
			String tween_id;
			Node *node = get_bound_node();
			if (node) {
				tween_id = vformat("Spreen (bound to %s)", node->is_inside_tree() ? (String)node->get_path() : (String)node->get_name());
			} else {
				tween_id = to_string();
			}
			ERR_FAIL_V_MSG(false, tween_id + ": started with no Spreeners.");
		}
		total_time = 0;
		_start_spreeners();
		started = true;
	}

  total_time += p_delta;

  bool still_active = false;
  for (Ref<Spreener> &spreener : spreeners) {
		still_active = spreener->step(p_delta) || still_active;
	}

  if (!still_active && finishable) {
    running = false;
    dead = true;
    emit_signal(SceneStringName(finished));
  }

  return true;
}

bool Spreen::can_process(bool p_tree_paused) const {
	if (is_bound && pause_mode == SPREEN_PAUSE_BOUND) {
		Node *node = get_bound_node();
		if (node) {
			return node->is_inside_tree() && node->can_process();
		}
	}

	return !p_tree_paused || pause_mode == SPREEN_PAUSE_PROCESS;
}

Node *Spreen::get_bound_node() const {
	if (is_bound) {
		return Object::cast_to<Node>(ObjectDB::get_instance(bound_node));
	} else {
		return nullptr;
	}
}

double Spreen::get_total_time() const {
	return total_time;
}

String Spreen::to_string() {
	String ret = Object::to_string();
	Node *node = get_bound_node();
	if (node) {
		ret += vformat(" (bound to %s)", node->get_name());
	}
	return ret;
}

void Spreen::_bind_methods() {
  ClassDB::bind_method(D_METHOD("spreen_float", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_float);
  ClassDB::bind_method(D_METHOD("spreen_vector2", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_vector2);
  ClassDB::bind_method(D_METHOD("spreen_transform_2d", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_transform_2d);
  ClassDB::bind_method(D_METHOD("spreen_vector3", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_vector3);
  ClassDB::bind_method(D_METHOD("spreen_basis", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_basis);
  ClassDB::bind_method(D_METHOD("spreen_transform_3d", "object", "property", "goal", "damping_ratio", "halflife"), &Spreen::spreen_transform_3d);

  ClassDB::bind_method(D_METHOD("custom_step", "delta"), &Spreen::custom_step);
	ClassDB::bind_method(D_METHOD("stop"), &Spreen::stop);
	ClassDB::bind_method(D_METHOD("pause"), &Spreen::pause);
	ClassDB::bind_method(D_METHOD("play"), &Spreen::play);
	ClassDB::bind_method(D_METHOD("kill"), &Spreen::kill);
	ClassDB::bind_method(D_METHOD("get_total_elapsed_time"), &Spreen::get_total_time);


	ClassDB::bind_method(D_METHOD("is_finishable"), &Spreen::is_finishable);
	ClassDB::bind_method(D_METHOD("is_running"), &Spreen::is_running);
	ClassDB::bind_method(D_METHOD("is_valid"), &Spreen::is_valid);
	ClassDB::bind_method(D_METHOD("bind_node", "node"), &Spreen::bind_node);
	ClassDB::bind_method(D_METHOD("set_process_mode", "mode"), &Spreen::set_process_mode);
	ClassDB::bind_method(D_METHOD("set_pause_mode", "mode"), &Spreen::set_pause_mode);
	ClassDB::bind_method(D_METHOD("set_finishable", "finishable"), &Spreen::set_finishable);

  ADD_SIGNAL(MethodInfo("finished"));

  BIND_ENUM_CONSTANT(SPREEN_PROCESS_PHYSICS);
	BIND_ENUM_CONSTANT(SPREEN_PROCESS_IDLE);

	BIND_ENUM_CONSTANT(SPREEN_PAUSE_BOUND);
	BIND_ENUM_CONSTANT(SPREEN_PAUSE_STOP);
	BIND_ENUM_CONSTANT(SPREEN_PAUSE_PROCESS);
}

Spreen::Spreen() {
	ERR_FAIL_MSG("Tween can't be created directly. Use create_tween() method.");
}

Spreen::Spreen(bool p_valid) {
	valid = p_valid;
}