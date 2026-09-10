plugins {
    id("java-library")
    id("io.deepmedia.tools.deployer")
    // RUCKUS PATCH: Dokka removed - see RUCKUS_PATCHES.md. Its consumable configurations were
    // being selected instead of runtimeElements when this java-library is consumed from the
    // Android app across the composite build, so core's classes never reached the APK.
    // Spotless removed with it: a formatter has no job in a vendored copy.
}

dependencies {
    testImplementation("org.junit.jupiter:junit-jupiter:5.9.3")
    testImplementation("com.google.truth:truth:1.4.5")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher:1.9.3")
}

java {
    sourceCompatibility = JavaVersion.VERSION_1_8
    targetCompatibility = JavaVersion.VERSION_1_8
}

tasks.test {
    useJUnitPlatform()
}

deployer {
    projectInfo {
        name = "Pedro Pathing Core"
        description = "A path follower designed to revolutionize autonomous pathing in robotics"
        url = "https://pedropathing.com"
        scm {
            fromGithub("Pedro-Pathing", "PedroPathing")
        }
        license("BSD 3-Clause License", "https://opensource.org/licenses/BSD-3-Clause")

        developer("Baron Henderson", "baron@pedropathing.com")
        developer("Havish Sripada", "havish@pedropathing.com")
        developer("Davis Luxenberg", "davis@pedropathing.com")
    }

    content {
        component {
            fromJava()
            javaSources()
            // RUCKUS PATCH: docs(dokkaJar) removed along with the Dokka plugin.
        }
    }

    if (System.getenv("PUBLISH_PEDRO") == "yes please") {
        signing {
            key = secret("MVN_GPG_KEY")
            password = secret("MVN_GPG_PASSWORD")
        }

        centralPortalSpec {
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
            allowMavenCentralSync = false
        }

        nexusSpec("snapshot") {
            repositoryUrl = "https://central.sonatype.com/repository/maven-snapshots/"
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
        }
    }

    localSpec()
}
