#include "register_types.h"

#include "enemy_bodies.h"
#include "enemy_replicas.h"
#include "enemy_views.h"
#include "enemy_world.h"

#include "core/object/class_db.h"

void initialize_enemy_world_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(EnemyWorld);
		GDREGISTER_CLASS(EnemyViews);
		GDREGISTER_CLASS(EnemyBodies);
		GDREGISTER_CLASS(EnemyReplicas);
	}
}

void uninitialize_enemy_world_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
