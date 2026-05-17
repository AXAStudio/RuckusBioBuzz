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

As of 2026-05-17, there are no intentional local source patches inside `third_party/PedroPathing`.

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
