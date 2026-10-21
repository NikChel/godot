#include "register_types.h"

#include "enemy_world.h"

#include "core/object/class_db.h"

void initialize_enemy_world_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(EnemyWorld);
	}
}

void uninitialize_enemy_world_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
