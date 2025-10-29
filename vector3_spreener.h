//===============================================================================//
// Spreen - vector3_spreener.h
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

#ifndef VECTOR3_SPREENER
#define VECTOR3_SPREENER

#include "spreen.h"

class Vector3Spreener : public Spreener {
	GDCLASS(Vector3Spreener, Spreener);

public:
	void start() override;
	bool step(double &r_delta) override;
	Ref<Vector3Spreener> update_goal(const Vector3 &p_goal);
	Ref<Vector3Spreener> set_damping_ratio(real_t p_damping_ratio);
	Ref<Vector3Spreener> set_halflife(real_t p_halflife);

	Vector3Spreener(const Object *p_target, const Vector<StringName> &p_property, const Vector3 &p_goal, real_t p_damping_ratio, real_t p_halflife);
	Vector3Spreener();

protected:
	static void _bind_methods();

private:
	Vector3 goal;
	Vector3 velocity;
};

#endif // VECTOR3_SPREENER
