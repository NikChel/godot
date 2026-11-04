#include "register_types.h"

#include "ragdoll_sync.h"

#include "core/object/class_db.h"

void initialize_ragdoll_sync_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(RagdollSync);
	}
}

void uninitialize_ragdoll_sync_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
