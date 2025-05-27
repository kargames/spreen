//===============================================================================//
// Spreen - spreen.h
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

#ifndef SPREEN_H
#define SPREEN_H

#include "core/object/ref_counted.h"

class Spreen;
class Node;

class Spreener : public RefCounted {
	GDCLASS(Spreener, RefCounted);

	ObjectID spreen_id;

public:
	virtual void set_spreen(const Ref<Spreen> &p_spreen);
	virtual void start() = 0;
	virtual bool step(double &r_delta) = 0;

  real_t get_damping_ratio() const;
  real_t get_halflife() const;

protected:
	static void _bind_methods();

  ObjectID target;
	Vector<StringName> property;
  real_t damping_ratio = 0.5f;
  real_t halflife = 0.5f;

  void update_spring(real_t &x, real_t &v, const real_t x_goal, real_t v_goal, double delta);
  void update_spring(Vector2 &p, Vector2 &v, const Vector2 &p_goal, const Vector2 &v_goal, double delta);
  void update_spring(Vector3 &p, Vector3 &v, const Vector3 &p_goal, const Vector3 &v_goal, double delta);
  void update_spring(Quaternion &q, Vector3 &v, const Quaternion &q_goal, double delta);

  Ref<RefCounted> ref_copy; // Makes sure that RefCounted objects are not freed too early.

	Ref<Spreen> _get_spreen();
  void _finish();

	double elapsed_time = 0;
	bool finished = false;

  real_t halflife_to_damping(real_t p_halflife) {
    return (4.0 * 0.69314718056) / (p_halflife + CMP_EPSILON);
  }

  real_t damping_ratio_to_stiffness(real_t p_ratio, real_t p_damping) {
    return Math::pow(p_damping / (p_ratio * 2.0), 2.0);
  }

  /* These next functions feel like they could be replaced with existing Godot Quaternion functions but I have had little success so far */
  Quaternion quat_exp(const Vector3 &v)
  {
    real_t halfangle = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

    if (halfangle < CMP_EPSILON)
    {
      return Quaternion(v.x, v.y, v.z, 1.).normalized();
    }
    else
    {
      real_t c = Math::cos(halfangle);
      real_t s = Math::sin(halfangle) / halfangle;
      return Quaternion(s * v.x, s * v.y, s * v.z, c);
    }
  }

  Vector3 quat_log(const Quaternion &q)
  {
    real_t length = Math::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);

    if (length < CMP_EPSILON)
    {
      return Vector3(q.x, q.y, q.z);
    }
    else
    {
      real_t halfangle = Math::acos(CLAMP(q.w, -1.0, 1.0));
      return halfangle * (Vector3(q.x, q.y, q.z) / length);
    }
  }

  Quaternion quat_from_scaled_angle_axis(const Vector3 &v)
  {
    return quat_exp(v / 2.0);
  }

  Vector3 quat_to_scaled_angle_axis(const Quaternion &q)
  {
    return 2.0 * quat_log(q);
  }

  Quaternion quat_abs(const Quaternion &q)
  {
    return q.w < 0.0 ? -q : q;
  }
};

class FloatSpreener;
class Vector2Spreener;
class Transform2DSpreener;
class Vector3Spreener;
class BasisSpreener;
class Transform3DSpreener;

class Spreen : public RefCounted {
  GDCLASS(Spreen, RefCounted);

  friend class FloatSpreener;

public:
  enum SpreenProcessMode {
    SPREEN_PROCESS_PHYSICS,
    SPREEN_PROCESS_IDLE,
  };

  enum SpreenPauseMode {
		SPREEN_PAUSE_BOUND,
		SPREEN_PAUSE_STOP,
		SPREEN_PAUSE_PROCESS,
	};

private:
  SpreenProcessMode process_mode = SpreenProcessMode::SPREEN_PROCESS_IDLE;
  SpreenPauseMode pause_mode = SpreenPauseMode::SPREEN_PAUSE_BOUND;
  ObjectID bound_node;

  List<Ref<Spreener>> spreeners;
  double total_time = 0;
  
  bool finishable = true;
  bool is_bound = false;
  bool started = false;
  bool running = true;
  bool dead = false;
  bool valid = false;

  void _start_spreeners();
  void _stop_internal(bool p_reset);

protected:
	static void _bind_methods();

public:
	virtual String to_string() override;

  Ref<FloatSpreener> spreen_float(const Object *p_target, const NodePath &p_property, real_t p_goal, real_t p_damping_ratio, real_t p_halflife);
  Ref<Vector2Spreener> spreen_vector2(const Object *p_target, const NodePath &p_property, const Vector2 &p_goal, real_t p_damping_ratio, real_t p_halflife);
  Ref<Transform2DSpreener> spreen_transform_2d(const Object *p_target, const NodePath &p_property, const Transform2D &p_goal, real_t p_damping_ratio, real_t p_halflife);
  Ref<Vector3Spreener> spreen_vector3(const Object *p_target, const NodePath &p_property, const Vector3 &p_goal, real_t p_damping_ratio, real_t p_halflife);
  Ref<BasisSpreener> spreen_basis(const Object *p_target, const NodePath &p_property, const Basis &p_goal, real_t p_damping_ratio, real_t p_halflife);
  Ref<Transform3DSpreener> spreen_transform_3d(const Object *p_target, const NodePath &p_property, const Transform3D &p_goal, real_t p_damping_ratio, real_t p_halflife);
  void append(Ref<Spreener> p_spreener);

  bool custom_step(double p_delta);
	void stop();
	void pause();
	void play();
	void kill();

  bool is_finishable();
  bool is_running();
	bool is_valid();
	void clear();

  Ref<Spreen> bind_node(const Node *p_node);
	Ref<Spreen> set_process_mode(SpreenProcessMode p_mode);
	SpreenProcessMode get_process_mode();
	Ref<Spreen> set_pause_mode(SpreenPauseMode p_mode);
	SpreenPauseMode get_pause_mode();
  Ref<Spreen> set_finishable(bool p_finishable);

  bool step(double p_delta);
	bool can_process(bool p_tree_paused) const;
  Node *get_bound_node() const;
	double get_total_time() const;

  Spreen();
	Spreen(bool p_valid);
};

VARIANT_ENUM_CAST(Spreen::SpreenProcessMode)
VARIANT_ENUM_CAST(Spreen::SpreenPauseMode)

#endif // SPREEN_H