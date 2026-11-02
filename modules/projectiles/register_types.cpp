#include "register_types.h"

#include "projectile_swarm.h"
#include "projectile_views.h"

#include "core/object/class_db.h"

void initialize_projectiles_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(ProjectileSwarm);
		GDREGISTER_CLASS(ProjectileViews);
	}
}

void uninitialize_projectiles_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}
