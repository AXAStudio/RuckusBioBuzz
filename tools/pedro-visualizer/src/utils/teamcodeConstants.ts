/**
 * Reads the robot's Pedro Pathing 3 constants out of TeamCode.
 *
 * Every number driving the time estimate, the velocity gradient and the
 * clearance margins was typed in by hand, which makes the whole model only as
 * good as eight guesses. Pedro 3's Foresight follower is configured with
 * physical quantities — top speed, how fast the robot coasts down, an optional
 * acceleration cap — and they sit in the drivetrain constants file next to this
 * tool. Reading them beats copying numbers between two files.
 *
 * Two things matter as much as what is read. Some of those numbers are
 * placeholders until the Foresight Tuner has been run, and the file says so:
 * `SwerveDrivetrainConstants.FORESIGHT_MEASURED`, and constants named
 * `PLACEHOLDER_*`. A placeholder is never offered as a measurement. And some
 * settings Pedro simply has no number for; those stay visibly hand-tuned.
 */

export type Drivetrain = "swerve" | "mecanum";

/** A value read from the constants file, and where it came from. */
export interface ImportedConstant {
  /** The `Settings` field this lands on. */
  setting: string;
  label: string;
  value: number;
  /** The Java call it was read from, for showing the provenance. */
  source: string;
  /** Units, for display. */
  unit: string;
  /**
   * True when the number is an upper bound on what the follower achieves, so
   * applying it straight makes the estimate optimistic.
   */
  ceiling?: boolean;
  /**
   * True when the number is a lower bound - the follower does at least this -
   * so applying it straight makes the estimate pessimistic.
   */
  floor?: boolean;
  note?: string;
}

export interface TeamCodeConstants {
  drivetrain: Drivetrain;
  /** The file the values came from, repo-relative. */
  sourceFile: string;
  /**
   * The file's own verdict on its Foresight constants: `FORESIGHT_MEASURED`.
   * Null when the file does not declare it.
   */
  foresightMeasured: boolean | null;
  values: ImportedConstant[];
  /** Settings the constants file has no answer for, and why. */
  missing: Array<{ setting: string; label: string; reason: string }>;
}

/** Fields Pedro 3 genuinely does not describe, so they stay hand-tuned. */
const NOT_IN_PEDRO: TeamCodeConstants["missing"] = [
  {
    setting: "aVelocity",
    label: "Max turn rate",
    reason:
      "Foresight states no turn-rate limit. Its heading brake coefficients describe how far a spin carries on, not how fast the robot can spin.",
  },
  {
    setting: "turnCoupling",
    label: "Turn coupling",
    reason:
      "How much turning eats into driving speed is this tool's own model; Pedro has no equivalent.",
  },
  {
    setting: "maxLateralAcceleration",
    label: "Cornering grip",
    reason:
      "Foresight has no cornering or centripetal limit, so there is no grip figure to read.",
  },
];

/**
 * Removes comments so a commented-out call is never read as live.
 *
 * These files carry alternates parked behind `//`, and picking one of those up
 * would silently import a number the robot is not using.
 */
export function stripComments(source: string): string {
  return source
    .replace(/\/\*[\s\S]*?\*\//g, " ")
    .replace(/(^|[^:])\/\/[^\n]*/g, "$1");
}

/** Which drivetrain `config.jsonc` selects. */
export function readDrivetrain(configJsonc: string): Drivetrain | null {
  const match = /"drivetrain"\s*:\s*"([a-z]+)"/i.exec(stripComments(configJsonc));
  const value = match?.[1]?.toLowerCase();
  return value === "swerve" || value === "mecanum" ? value : null;
}

/** A `c.<field>.set(<argument>)` read out of a config lambda. */
export interface ConfigSetting {
  /** The argument exactly as written. */
  expression: string;
  /** Its value, when it is a number or a numeric constant in the same file. */
  value: number | null;
  /** True when the argument is a constant named as a placeholder. */
  placeholder: boolean;
}

/**
 * Pulls a Pedro 3 config field out of Java source: `c.<field>.set(<argument>)`.
 *
 * A regex rather than a parser because these lambdas always take this shape;
 * anything cleverer would be more to go wrong for no benefit. An argument that
 * names a `static final double` in the same file is resolved through it, which
 * is how the constants files keep one number in one place.
 */
export function readConfigSetting(source: string, field: string): ConfigSetting | null {
  const code = stripComments(source);
  const pattern = new RegExp(`\\.\\s*${field}\\s*\\.\\s*set\\s*\\(\\s*([^;]*?)\\s*\\)\\s*;`);
  const match = pattern.exec(code);
  if (!match) return null;

  const expression = match[1].trim();
  return {
    expression,
    value: resolveNumber(code, expression),
    placeholder: /\bPLACEHOLDER/i.test(expression),
  };
}

function resolveNumber(code: string, expression: string, depth = 0): number | null {
  const literal = Number(expression.replace(/[dDfF]$/, ""));
  if (expression !== "" && Number.isFinite(literal)) return literal;
  if (depth > 3 || !/^[A-Za-z_][A-Za-z0-9_]*$/.test(expression)) return null;

  const declaration = new RegExp(
    `static\\s+final\\s+double\\s+${expression}\\s*=\\s*([^;]+);`,
  ).exec(code);
  return declaration ? resolveNumber(code, declaration[1].trim(), depth + 1) : null;
}

/** `FORESIGHT_MEASURED`, when the file declares it. */
export function readForesightMeasured(source: string): boolean | null {
  const match = /\bFORESIGHT_MEASURED\s*=\s*(true|false)\s*;/.exec(stripComments(source));
  return match ? match[1] === "true" : null;
}

/**
 * Reads what the constants file can tell us about how the robot moves.
 *
 * `maxAchievableForwardVelocity` is Foresight's top speed. The natural
 * deceleration is how fast the robot slows with no power applied; Foresight
 * brakes actively on top of that, so as a braking figure it is a floor and the
 * estimate leans pessimistic. (Pedro 2's zero-power acceleration was the same
 * physical quantity, and this tool used to call it a ceiling - it never was.)
 */
export function parseTeamCodeConstants(
  drivetrain: Drivetrain,
  source: string,
  sourceFile: string,
): TeamCodeConstants {
  const values: ImportedConstant[] = [];
  const missing = [...NOT_IN_PEDRO];
  const foresightMeasured = readForesightMeasured(source);

  /**
   * Adds a setting when the file has a real number for it, and otherwise says
   * why not - a placeholder is reported, never imported.
   */
  const take = (
    setting: string,
    label: string,
    field: string,
    unit: string,
    extra: Partial<ImportedConstant> = {},
  ) => {
    const read = readConfigSetting(source, field);
    const source_ = `foresightConfig.${field}.set(${read?.expression ?? ""})`;

    if (!read) {
      missing.unshift({ setting, label, reason: `No \`c.${field}.set(...)\` found.` });
      return;
    }
    if (read.placeholder) {
      missing.unshift({
        setting,
        label,
        reason: `TeamCode carries a placeholder here (${read.expression}${read.value !== null ? ` = ${round(read.value)} ${unit}` : ""}), not a measurement. Run the Foresight Tuner.`,
      });
      return;
    }
    if (read.value === null || !(Math.abs(read.value) > 0) || !Number.isFinite(read.value)) {
      missing.unshift({
        setting,
        label,
        reason: `\`${source_}\` is not a number this tool can read.`,
      });
      return;
    }
    values.push({ setting, label, value: round(Math.abs(read.value)), source: source_, unit, ...extra });
  };

  take("maxVelocity", "Max velocity", "maxAchievableForwardVelocity", "in/s", {
    note:
      drivetrain === "mecanum"
        ? "Forward speed. A mecanum is slower sideways, and this tool profiles one speed, so the paths are timed as if driven forward."
        : undefined,
  });

  take("maxDeceleration", "Max deceleration", "naturalForwardDeceleration", "in/s²", {
    floor: true,
    note:
      "How fast the robot slows with no power applied. Foresight brakes actively on top of that, so it stops at least this hard and the estimate leans pessimistic.",
  });

  // Foresight's acceleration cap defaults to Constraint.NONE (unlimited).
  const acceleration = readConfigSetting(source, "maxAccelerationConstraint");
  if (acceleration && !acceleration.placeholder && acceleration.value !== null && acceleration.value > 0) {
    values.push({
      setting: "maxAcceleration",
      label: "Max acceleration",
      value: round(acceleration.value),
      source: `foresightConfig.maxAccelerationConstraint.set(${acceleration.expression})`,
      unit: "in/s²",
    });
  } else {
    missing.unshift({
      setting: "maxAcceleration",
      label: "Max acceleration",
      reason:
        "Foresight's maxAccelerationConstraint is unset (Constraint.NONE): it accelerates as hard as the drivetrain allows, so there is no number to read.",
    });
  }

  return { drivetrain, sourceFile, foresightMeasured, values, missing };
}

/** Extra readings shown for context but not applied to any setting. */
export interface TeamCodeReference {
  label: string;
  value: number;
  unit: string;
  source: string;
}

export function parseTeamCodeReference(source: string): TeamCodeReference[] {
  const reference: TeamCodeReference[] = [];

  const add = (label: string, field: string, unit: string) => {
    const read = readConfigSetting(source, field);
    if (!read || read.value === null) return;
    reference.push({
      label: read.placeholder ? `${label} (placeholder)` : label,
      value: round(Math.abs(read.value)),
      unit,
      source: `foresightConfig.${field}`,
    });
  };

  add("Sideways speed", "maxAchievableStrafeVelocity", "in/s");
  add("Sideways coast-down", "naturalStrafeDeceleration", "in/s²");
  add("Braking power cap", "maxBrakingPower", "");

  return reference;
}

function round(value: number): number {
  return Math.round(value * 100) / 100;
}

/** The change one imported constant would make, for showing before applying. */
export interface ConstantChange extends ImportedConstant {
  current: number | undefined;
  changed: boolean;
}

export function diffAgainstSettings(
  imported: ImportedConstant[],
  settings: Record<string, unknown>,
): ConstantChange[] {
  return imported.map((entry) => {
    const current = Number(settings?.[entry.setting]);
    const has = Number.isFinite(current);
    return {
      ...entry,
      current: has ? current : undefined,
      changed: !has || Math.abs(current - entry.value) > 1e-6,
    };
  });
}
