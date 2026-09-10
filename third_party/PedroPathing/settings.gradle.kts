pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

@Suppress("UnstableApiUsage")
dependencyResolutionManagement {
    repositoriesMode = RepositoriesMode.FAIL_ON_PROJECT_REPOS
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "PedroPathing"
include(":core", ":revhub")
// RUCKUS PATCH: local module, not part of upstream PedroPathing - see RUCKUS_PATCHES.md.
include(":telemetry")
