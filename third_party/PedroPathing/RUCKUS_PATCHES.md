# Ruckus PedroPathing Patch Notes

This folder is vendored into the main `RuckusBioBuzz` git repository. It is not a nested git checkout or submodule.

## Upstream

Official source:

```bash
https://github.com/Pedro-Pathing/PedroPathing.git
```

The main repository should have a remote named `pedro-upstream`:

```bash
git remote add pedro-upstream https://github.com/Pedro-Pathing/PedroPathing.git
git remote set-url --push pedro-upstream DISABLED
```

The vendored Gradle metadata currently reports PedroPathing version `2.1.2`.

## Local Patches

### 2026-08-11 — Make the integral term safe to use in `PIDFController`

- **Files changed:** `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** The swerve pods park 2–4° short of target on carpet and stop dead. `CoaxialPod.move()`
  zeroes the feed-forward inside 2°, and P alone there is far below breakaway, so nothing remains to
  close the gap. Raising kF makes it worse — the pod breaks free, overshoots and stops on the far
  side. An integral term is the correct fix, but stock `PIDFController` could not carry one safely:
  `errorIntegral` accumulates without bound, is never clamped, and nothing calls `reset()` during
  normal operation, so any non-zero I eventually saturates the servo.

  Two changes, both no-ops while I is 0 — which is every stock Pedro config:

  1. `run()` clamps the integral **contribution** to `integralLimit` (default 0.25 of normalised
     output). Clamping the contribution rather than the raw sum keeps the bound meaningful for any I.
  2. `updateError()` and `updatePosition()` zero `errorIntegral` when the error changes sign.
     Accumulation from the approach is stale once the target is crossed, and carrying it through a
     swerve pod's 180° flip would drive hard the wrong way.

- **Upstream issue:** none filed. Arguably worth proposing upstream — unbounded integral with no
  reset path is a latent trap for anyone who sets I.
- **How to remove:** if upstream adds its own anti-windup, drop both patches and use theirs. Search
  the file for `RUCKUS PATCH`. Verify afterwards that a pod with I set does not creep or saturate
  when held off-target.

### 2026-08-09 — Remove Dokka from `:core` so its classes reach the APK

- **Files changed:** `core/build.gradle.kts`
- **Reason:** With Dokka applied, `:core` exposes consumable configurations whose attributes are
  `DGP~`-prefixed strings rather than Gradle's typed attributes. When the Android app consumed
  `:core` transitively (via `ftc`'s `api(project(":core"))`) across the composite build, Gradle
  selected the variant `dokkaHtmlPublicationPluginApiOnlyConsumable~internal` instead of
  `runtimeElements`. That variant carries the Dokka documentation-plugin classpath, not the code
  jar, so **no `com.pedropathing` core class was packaged into the APK** — verified with `dexdump`:
  `follower`, `geometry`, `control`, `math` were all absent while `com.pedropathing.ftc.*` was
  present.

  The symptom was a Robot Controller crash loop: Panels' Configurables plugin scans `@Configurable`
  classes at startup, and `getDeclaredFields()` on `Tuning` threw
  `NoClassDefFoundError: Lcom/pedropathing/follower/Follower;`, taking down the whole app process
  every ~40 seconds. That killed FTC Dashboard connections and made every Pedro OpMode unusable.

  Removed the `org.jetbrains.dokka` plugin, its `dokkaPlugin` dependency, the `dokkaJar` task and
  the `docs(dokkaJar)` deployer entry. Generated documentation has no value for a vendored copy.

- **Upstream issue:** none filed; this is a Dokka Gradle Plugin v2 variant-selection interaction
  with composite builds, not a PedroPathing source bug.
- **How to remove:** once DGP no longer publishes those configurations for `java-library` consumers
  (or if Pedro is consumed from Maven instead of `includeBuild`), restore the four removed pieces
  from upstream `core/build.gradle.kts`. After any change here, confirm with:

  ```bash
  ./gradlew :TeamCode:dependencyInsight --configuration debugRuntimeClasspath --dependency core
  ```

  The selected variant must be `runtimeElements`, not a `dokka*` one.

When local Pedro changes are added, list them here with:

- files changed
- reason for the patch
- upstream issue or PR if one exists
- how to remove the patch once upstream includes it

## Upstream Update Workflow

Fetch upstream tags and branches without touching the working tree:

```bash
git fetch pedro-upstream --tags
```

Review upstream changes before applying them:

```bash
git log --oneline v2.1.2..pedro-upstream/main
git diff --stat v2.1.2..pedro-upstream/main
```

Because Pedro is vendored under `third_party/PedroPathing`, do not merge `pedro-upstream/main` directly into the robot repo. Apply upstream changes into the vendored prefix from the recorded base tag, then compile TeamCode before committing.

Recommended update shape, replacing `main` with a newer tag if you want a tagged release instead:

```bash
git diff --binary v2.1.2..pedro-upstream/main -- . \
  | git apply --3way --directory=third_party/PedroPathing
```

If upstream changes conflict with local patches, resolve the files in `third_party/PedroPathing`, update this note, then run the normal TeamCode Gradle compile. After a successful update, replace the base tag in this file with the new upstream tag or commit SHA.
