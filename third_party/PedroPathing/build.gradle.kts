plugins {
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.deployer) apply false
    // RUCKUS PATCH: dokka, kotlin-android and spotless removed - see RUCKUS_PATCHES.md. None is
    // applied by anything this composite build compiles, and resolving them still pulls plugin
    // versions that have not been checked against the root project's Gradle.
}

subprojects {
    group = "com.pedropathing"
    version = property("version") as String
}

if (System.getenv("PUBLISH_PEDRO") == "yes please") {
    tasks.register("deployCentralPortal") {
        group = "publishing"
        description = "Publishes all subprojects to Maven Central."
        dependsOn(subprojects.map { it.tasks.named("deployCentralPortal") })
    }

    tasks.register("deployNexusSnapshot") {
        group = "publishing"
        description = "Publishes all subprojects to Maven Central Snapshots."
        dependsOn(subprojects.map { it.tasks.named("deployNexusSnapshot") })
    }
}

tasks.register("deployLocal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Local."
    dependsOn(subprojects.map { it.tasks.named("deployLocal") })
}
