//===============================================================================//
// Spreen - spreen_tree.h
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

#ifndef SPREEN_TREE_H
#define SPREEN_TREE_H

#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/os/thread_safe.h"

class Node;
class Spreen;

class SpreenTree : public Object {
	_THREAD_SAFE_CLASS_

	GDCLASS(SpreenTree, Object);

private:
	static SpreenTree *singleton;
	bool initialized = false;
	List<Ref<Spreen>> spreens;

	void _initialize();

protected:
	static void _bind_methods();

public:
	void physics_process();
	void process();
	void process_spreens(double p_delta, bool p_physics_frame);

	Ref<Spreen> create_spreen(const Node *p_node = nullptr);

	static SpreenTree *get_singleton() { return singleton; }

	SpreenTree();
	~SpreenTree();
};

#endif // SPREEN_TREE_H
