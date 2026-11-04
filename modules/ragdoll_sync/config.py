def can_build(env, platform):
    env.module_add_dependencies("ragdoll_sync", ["projectiles"])  # ViewLag (modules/projectiles/view_lag.h)
    return True


def configure(env):
    pass
