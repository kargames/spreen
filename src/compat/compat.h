//===============================================================================//
// Spreen - compat/compat.h
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
//
// Bridges the API differences between an engine module and GDExtension build
//
//===============================================================================//

#ifndef SPREEN_COMPAT_H
#define SPREEN_COMPAT_H

#ifdef GDEXTENSION

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/core/object_id.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/templates/mutex.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

#else

#include "core/config/engine.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/object/ref_counted.h"
#include "core/os/thread_safe.h"
#include "core/string/node_path.h"
#include "core/templates/list.h"
#include "core/variant/type_info.h"
#include "core/variant/variant.h"
#include "scene/main/node.h"
#include "scene/main/scene_tree.h"
#include "scene/main/window.h"
#include "scene/scene_string_names.h"

#endif

#ifdef GDEXTENSION

#define SPREEN_THREAD_SAFE_CLASS mutable Mutex _spreen_mutex;
#define SPREEN_THREAD_SAFE_METHOD MutexLock _spreen_lock(_spreen_mutex);
#define SPREEN_SNAME_FINISHED StringName("finished")
#define SPREEN_TO_STRING_DECL String _to_string() const
#define SPREEN_TO_STRING_DEF(m_class) String m_class::_to_string() const
#define ADD_PROPERTY_DEFAULT(m_name, m_default) ((void)0)

#else

#define SPREEN_THREAD_SAFE_CLASS _THREAD_SAFE_CLASS_
#define SPREEN_THREAD_SAFE_METHOD _THREAD_SAFE_METHOD_
#define SPREEN_SNAME_FINISHED SceneStringName(finished)
#define SPREEN_TO_STRING_DECL virtual String _to_string() override
#define SPREEN_TO_STRING_DEF(m_class) String m_class::_to_string()

#endif

// SceneTree::get_process_time is not exposed to GDExtension
_FORCE_INLINE_ double spreen_process_delta(SceneTree *p_tree) {
#ifdef GDEXTENSION
	Window *root = p_tree->get_root();
	return root ? root->get_process_delta_time() : 0.0;
#else
	return p_tree->get_process_time();
#endif
}

// SceneTree::get_physics_process_time is not exposed to GDExtension
_FORCE_INLINE_ double spreen_physics_delta(SceneTree *p_tree) {
#ifdef GDEXTENSION
	Window *root = p_tree->get_root();
	return root ? root->get_physics_process_delta_time() : 0.0;
#else
	return p_tree->get_physics_process_time();
#endif
}

// get_indexed varies between core and GDExtension
_FORCE_INLINE_ Variant spreen_get_indexed(const Object *p_object, const NodePath &p_path, bool *r_valid = nullptr) {
#ifdef GDEXTENSION
	Variant value = p_object->get_indexed(p_path);
	if (r_valid) {
		*r_valid = value.get_type() != Variant::NIL;
	}
	return value;
#else
	return p_object->get_indexed(p_path.get_subnames(), r_valid);
#endif
}

// set_indexed varies between core and GDExtension
_FORCE_INLINE_ void spreen_set_indexed(Object *p_object, const NodePath &p_path, const Variant &p_value) {
#ifdef GDEXTENSION
	p_object->set_indexed(p_path, p_value);
#else
	p_object->set_indexed(p_path.get_subnames(), p_value);
#endif
}

#endif // SPREEN_COMPAT_H
