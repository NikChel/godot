#include "register_types.h"

#include "frame_carry.h"

#include "core/object/class_db.h"

void initialize_frame_carry_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(FrameCarry);
	}
}

void uninitialize_frame_carry_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
