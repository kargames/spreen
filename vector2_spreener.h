//===============================================================================//
// Spreen - vector2_spreener.h
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

#ifndef VECTOR2_SPREENER
#define VECTOR2_SPREENER

#include "spreen.h"

class Vector2Spreener : public Spreener {
  GDCLASS(Vector2Spreener, Spreener);

public:
  void start() override;
  bool step(double &r_delta) override;
  Ref<Vector2Spreener> update_goal(const Vector2 &p_goal);
  Ref<Vector2Spreener> set_damping_ratio(real_t p_damping_ratio);
  Ref<Vector2Spreener> set_halflife(real_t p_halflife);

  Vector2Spreener(const Object *p_target, const Vector<StringName> &p_property, const Vector2 &p_goal, real_t p_damping_ratio, real_t p_halflife);
  Vector2Spreener();

protected:
	static void _bind_methods();

private:
  Vector2 goal;
  Vector2 velocity;
};

#endif // VECTOR2_SPREENER