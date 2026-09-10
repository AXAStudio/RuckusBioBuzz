plugins {
    id("com.android.library")
    id("io.deepmedia.tools.deployer")
    // RUCKUS PATCH: Dokka removed (see core/build.gradle.kts), and kotlin("android") with it -
    // this module has no Kotlin sources. See RUCKUS_PATCHES.md.
}

android {
    namespace = "com.pedropathing.revhub"
    // RUCKUS PATCH: upstream 35. TeamCode compiles at 34 and SDK platform 35 is not installed on
    // the build machine; nothing in this module needs an API above 24.
    compileSdk = 34

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    compileOnly(libs.bundles.ftc)
    api(project(":core"))
}

deployer {
    projectInfo {
        name = "Pedro Pathing FTC"
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
        androidComponents("release") {
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
