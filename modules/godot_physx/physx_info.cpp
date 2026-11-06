/**************************************************************************/
/*  physx_info.cpp                                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/


#include "physx_info.h"

#include "godot_physx_server_3d.h"
#include "objects/godot_physx_body_3d.h"

#include "core/object/class_db.h"

bool PhysX::is_gpu_available() {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	return server != nullptr && server->has_gpu();
}

String PhysX::get_gpu_device_name() {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	return server != nullptr ? server->get_gpu_device_name() : String();
}

void PhysX::body_set_contact_force_limit(RID p_body, real_t p_newtons) {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	ERR_FAIL_NULL_MSG(server, "PhysX is not the active physics engine.");
	GodotPhysXBody3D *body = server->get_body(p_body);
	ERR_FAIL_NULL(body);
	body->set_contact_force_limit(p_newtons);
}

real_t PhysX::body_get_contact_force_limit(RID p_body) {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V_MSG(server, 0.0, "PhysX is not the active physics engine.");
	const GodotPhysXBody3D *body = server->get_body(p_body);
	ERR_FAIL_NULL_V(body, 0.0);
	return body->get_contact_force_limit();
}

void PhysX::_bind_methods() {
	ClassDB::bind_static_method("PhysX", D_METHOD("is_gpu_available"), &PhysX::is_gpu_available);
	ClassDB::bind_static_method("PhysX", D_METHOD("get_gpu_device_name"), &PhysX::get_gpu_device_name);
	ClassDB::bind_static_method("PhysX", D_METHOD("body_set_contact_force_limit", "body", "newtons"), &PhysX::body_set_contact_force_limit);
	ClassDB::bind_static_method("PhysX", D_METHOD("body_get_contact_force_limit", "body"), &PhysX::body_get_contact_force_limit);
}
