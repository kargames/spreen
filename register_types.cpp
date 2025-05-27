//===============================================================================//
// Spreen - register_types.cpp
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

#include "register_types.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "spreen_tree.h"
#include "spreen.h"
#include "basis_spreener.h"
#include "float_spreener.h"
#include "transform_2d_spreener.h"
#include "transform_3d_spreener.h"
#include "vector2_spreener.h"
#include "vector3_spreener.h"

static SpreenTree *SpreenTreePtr = NULL;

void initialize_spreen_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    GDREGISTER_CLASS(SpreenTree);
    GDREGISTER_CLASS(Spreen);
    GDREGISTER_ABSTRACT_CLASS(Spreener);
    GDREGISTER_CLASS(FloatSpreener);
    GDREGISTER_CLASS(Vector2Spreener);
    GDREGISTER_CLASS(Transform2DSpreener);
    GDREGISTER_CLASS(Vector3Spreener);
    GDREGISTER_CLASS(BasisSpreener);
    GDREGISTER_CLASS(Transform3DSpreener);

    SpreenTreePtr = memnew(SpreenTree);
    Engine::get_singleton()->add_singleton(
            Engine::Singleton("SpreenTree", SpreenTree::get_singleton()));
}

void uninitialize_spreen_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    memdelete(SpreenTreePtr);
}
