plugins {
    id("com.android.library")
}

android {
    namespace = "com.pedropathing.telemetry"
    compileSdk = 30

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    defaultConfig {
        minSdk = 21
    }
}

dependencies {
    compileOnly(libs.bundles.ftc)
    compileOnly(libs.annotations)
}
