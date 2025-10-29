//===============================================================================//
// Spreen - spreen_tree.cpp
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

#include "spreen_tree.h"

#include "scene/main/node.h"
#include "scene/main/scene_tree.h"
#include "spreen.h"

// Store the singleton
SpreenTree *SpreenTree::singleton = NULL;

void SpreenTree::_initialize() {
	SceneTree *scene_tree = SceneTree::get_singleton();
	if (scene_tree) {
		Callable process_callback = callable_mp(SpreenTree::singleton, &SpreenTree::process);
		Callable physics_process_callback = callable_mp(SpreenTree::singleton, &SpreenTree::physics_process);
		scene_tree->connect("process_frame", process_callback);
		scene_tree->connect("physics_frame", physics_process_callback);
		initialized = true;
	}
}

void SpreenTree::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create_spreen", "bound_node"), &SpreenTree::create_spreen);
}

void SpreenTree::process() {
	process_spreens(SceneTree::get_singleton()->get_process_time(), false);
}

void SpreenTree::physics_process() {
	process_spreens(SceneTree::get_singleton()->get_physics_process_time(), true);
}

void SpreenTree::process_spreens(double p_delta, bool p_physics_frame) {
	_THREAD_SAFE_METHOD_
	// This methods works similarly to how SceneTreeTimers are handled.
	List<Ref<Spreen>>::Element *L = spreens.back();

	for (List<Ref<Spreen>>::Element *E = spreens.front(); E;) {
		List<Ref<Spreen>>::Element *N = E->next();
		// Don't process if paused or process mode doesn't match.
		SceneTree *scene_tree = SceneTree::get_singleton();
		bool is_paused = scene_tree ? scene_tree->is_paused() : false;
		if (!E->get()->can_process(is_paused) || (p_physics_frame == (E->get()->get_process_mode() == Spreen::SPREEN_PROCESS_IDLE))) {
			if (E == L) {
				break;
			}
			E = N;
			continue;
		}

		if (!E->get()->step(p_delta)) {
			E->get()->clear();
			spreens.erase(E);
		}
		if (E == L) {
			break;
		}
		E = N;
	}
}

Ref<Spreen> SpreenTree::create_spreen(const Node *p_node) {
	_THREAD_SAFE_METHOD_

	if (!initialized) {
		_initialize();
	}

	Ref<Spreen> spreen = memnew(Spreen(true));
	if (p_node != nullptr) {
		spreen->bind_node(p_node);
	}
	spreens.push_back(spreen);
	return spreen;
}

SpreenTree::SpreenTree() {
	singleton = this;
}

SpreenTree::~SpreenTree() {
	// This was causing issues because I think the SceneTree is already gone
	// if (initialized) {
	//   Callable process_callback = callable_mp(SpreenTree::singleton, &SpreenTree::process);
	//   Callable physics_process_callback = callable_mp(SpreenTree::singleton, &SpreenTree::physics_process);
	//   SceneTree::get_singleton()->disconnect("process_frame", process_callback);
	//   SceneTree::get_singleton()->disconnect("process_frame", physics_process_callback);
	// }
}
