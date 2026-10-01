package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Renders the tool's live tuning as declarations for the TUNED VALUES block at the top of
 * {@code pedroPathing/Constants.java}, one per line, so a bring-up session ends in a splice
 * ({@code tools/swervetune/robot.py constants --write}) rather than a transcription.
 *
 * <p>Every line has the exact shape of its declaration in Constants.java - the splicer replaces
 * by NAME, which leaves the file's provenance comments untouched. Everything the tool holds that
 * Constants.java cannot express (a per-pod pulse setting, a non-zero kI, a pod whose servo runs
 * the other way) is listed as a WARNING comment instead of being dropped silently: a tuned value
 * that vanishes in the paste is a robot that does not behave like the one that was tuned.
 */
public final class SwerveExport {

    private SwerveExport() {
    }

    /** Where Constants' factories put each servo number (leftFront() is ss2, and so on). */
    private static final String[] CORNER_BY_SS = {"RB", "RF", "LF", "LB"};

    /** The follower/localizer values the tool holds, read where the tool keeps them. */
    public static final class Live {
        /** Foresight's heading kP as the follower bench last ran it - NOT the tool's hold. */
        public double headingKP;
        public double toolHeadingHoldKP;
        public double toolHeadingHoldKD;
        public double forwardTranslationalPrimaryKP;
        public double forwardTranslationalSecondaryKP;
        public double strafeTranslationalPrimaryKP;
        public double strafeTranslationalSecondaryKP;
        public double translationalKI;
        public double translationalKD;
        public boolean epsilonTaper;
        public double demandSlewDegPerSec;
        /** {floor, velStartRadS, errGateRad}, CoaxialPod.getScheduleTuning(). */
        public double[] schedule;
        public double naturalForwardDeceleration;
        public double naturalStrafeDeceleration;
        public double pinpointXPodOffset;
        public double pinpointYPodOffset;
        public boolean pinpointXPodReversed;
        public boolean pinpointYPodReversed;
        public String label;
    }

    /**
     * @param bySs pods indexed by servo number ss0..ss3 - the indexing Constants' arrays use
     */
    public static String generate(PodCal[] bySs, Live live) {
        List<String> warnings = new ArrayList<>();
        StringBuilder sb = new StringBuilder();
        sb.append("// Swerve Bring-Up export");
        if (live.label != null && !live.label.isEmpty()) {
            sb.append(", run \"").append(live.label).append('"');
        }
        sb.append(".\n");
        sb.append("// Each line replaces the declaration of the same name in Constants.java's\n");
        sb.append("// TUNED VALUES block: python3 tools/swervetune/robot.py constants --write\n\n");

        // Only what the follower bench tunes (pedroPidf). Velocities, brake coefficients and
        // coast/brake kV come from Pedro's Foresight Tuner: robot.py foresight.
        sb.append("// Follower (Foresight) - as the follower bench last ran it (pedroPidf)\n");
        decl(sb, "double", "naturalForwardDeceleration", num(live.naturalForwardDeceleration, 1));
        decl(sb, "double", "naturalStrafeDeceleration", num(live.naturalStrafeDeceleration, 1));
        decl(sb, "double", "headingKP", num(live.headingKP, 2));
        decl(sb, "double", "forwardTranslationalPrimaryKP",
                num(live.forwardTranslationalPrimaryKP, 2));
        decl(sb, "double", "forwardTranslationalSecondaryKP",
                num(live.forwardTranslationalSecondaryKP, 2));
        decl(sb, "double", "strafeTranslationalPrimaryKP",
                num(live.strafeTranslationalPrimaryKP, 2));
        decl(sb, "double", "strafeTranslationalSecondaryKP",
                num(live.strafeTranslationalSecondaryKP, 2));
        decl(sb, "double", "translationalKI", num(live.translationalKI, 1));
        decl(sb, "double", "translationalKD", num(live.translationalKD, 3));
        if (live.toolHeadingHoldKP != live.headingKP) {
            warnings.add(String.format(Locale.US, "the tool's heading HOLD runs kP %.3f / kD %.4f; "
                    + "Foresight runs kP %.3f. They are different control laws - only the "
                    + "Foresight value is exported. Validate it with pedrocheck.py, not "
                    + "headingGoto.", live.toolHeadingHoldKP, live.toolHeadingHoldKD,
                    live.headingKP));
        }
        if (!live.epsilonTaper || live.demandSlewDegPerSec != 214.0) {
            warnings.add(String.format(Locale.US, "mixer is epsilon taper %s / demand slew %.0f "
                    + "deg/s; what ships is the vendored on / 214 (setMixer is a tool A/B "
                    + "toggle). Making it ship is a VENDORED change to Swerve.java.",
                    live.epsilonTaper ? "on" : "off", live.demandSlewDegPerSec));
        }
        if (live.schedule != null && (live.schedule[0] != 0.24
                || live.schedule[1] != Math.toRadians(22) || live.schedule[2] != Math.toRadians(20))) {
            warnings.add(String.format(Locale.US, "turn-gain schedule is floor %.3f, vel start %.1f "
                    + "deg/s, err gate %.1f deg; what ships is the vendored 0.24 / 22 / 20. "
                    + "Making it ship is a VENDORED change to CoaxialPod.java.", live.schedule[0],
                    Math.toDegrees(live.schedule[1]), Math.toDegrees(live.schedule[2])));
        }

        sb.append("\n// Localizer (Pinpoint), inches\n");
        decl(sb, "double", "pinpointXPodOffset", num(live.pinpointXPodOffset, 3));
        decl(sb, "double", "pinpointYPodOffset", num(live.pinpointYPodOffset, 3));
        decl(sb, "boolean", "pinpointXPodReversed", String.valueOf(live.pinpointXPodReversed));
        decl(sb, "boolean", "pinpointYPodReversed", String.valueOf(live.pinpointYPodReversed));

        int n = bySs.length;
        String[] kp = new String[n];
        String[] kd = new String[n];
        String[] ks = new String[n];
        String[] band = new String[n];
        String[] zero = new String[n];
        String[] minV = new String[n];
        String[] maxV = new String[n];
        String[] rev = new String[n];
        String[] enc = new String[n];
        for (int i = 0; i < n; i++) {
            PodCal p = bySs[i];
            String who = "ss" + i + " (" + p.label + ")";
            kp[i] = num(p.kP, 3);
            kd[i] = num(p.kD, 3);
            ks[i] = num(p.kS, 3);
            band[i] = num(p.kSBandDeg, 1);
            double z = Math.toDegrees(p.angleOffsetRad) % 360.0;
            zero[i] = num(z < 0 ? z + 360.0 : z, 5);
            minV[i] = num(p.analogMin, 3);
            maxV[i] = num(p.analogMax, 3);
            rev[i] = String.valueOf(p.driveDirection == DcMotorSimple.Direction.REVERSE);
            enc[i] = "\"" + p.encoderName + "\"";

            String corner = i < CORNER_BY_SS.length ? CORNER_BY_SS[i] : "?";
            if (!corner.equalsIgnoreCase(p.label)) {
                warnings.add(who + " is labelled " + p.label + " in the tool, but Constants' "
                        + "factories mount ss" + i + " at " + corner + ".");
            }
            if (!("sm" + i).equals(p.motorName) || !("ss" + i).equals(p.servoName)) {
                warnings.add(who + " is wired to motor \"" + p.motorName + "\" / servo \""
                        + p.servoName + "\"; Constants.buildPod uses sm" + i + "/ss" + i + ".");
            }
            if (p.servoDirection != DcMotorSimple.Direction.REVERSE) {
                warnings.add(who + " servo direction is " + p.servoDirection
                        + "; Constants.buildPod hardcodes REVERSE.");
            }
            if (p.encoderReversed) {
                warnings.add(who + " encoder is reversed; Constants.buildPod hardcodes false.");
            }
            if (p.kI != 0) {
                warnings.add(String.format(Locale.US, "%s kI %.4f: Constants.buildPod passes kI 0 "
                        + "and sets no integral band.", who, p.kI));
            }
            if (p.kF != 0) {
                warnings.add(String.format(Locale.US, "%s kF %.4f: shipped kF is 0 (it is a sign "
                        + "relay, see CLAUDE.md section 5).", who, p.kF));
            }
            if (p.derivativeOnMeasurement) {
                warnings.add(who + " uses derivative-on-measurement; Constants.buildPod does not.");
            }
            if (p.servoSlewPerUpdate != 0) {
                warnings.add(String.format(Locale.US, "%s servo slew limit %.4f/update: "
                        + "Constants.buildPod sets none.", who, p.servoSlewPerUpdate));
            }
            if (p.positional) {
                warnings.add(who + " is in POSITIONAL mode, which is shelved and not exportable.");
            }
        }
        sb.append("\n// Pods, indexed by servo number ss0..ss3 (0=RB, 1=RF, 2=LF, 3=LB)\n");
        decl(sb, "double[]", "turnKPPerPod", array(kp));
        decl(sb, "double[]", "turnKDPerPod", array(kd));
        decl(sb, "double[]", "turnKSPerPod", array(ks));
        decl(sb, "double[]", "turnKSBandDegPerPod", array(band));
        decl(sb, "double[]", "podZeroDeg", array(zero));
        decl(sb, "double[]", "podMinV", array(minV));
        decl(sb, "double[]", "podMaxV", array(maxV));
        decl(sb, "boolean[]", "podDriveReversed", array(rev));
        decl(sb, "String[]", "podEncoderNames", array(enc));

        // Scalars Constants.java shares across all four pods: exported only when the tool agrees
        // with itself, otherwise named and left alone.
        PodCal p0 = bySs[0];
        boolean cacheSame = true;
        boolean pulseSame = true;
        for (PodCal p : bySs) {
            cacheSame &= p.servoCaching == p0.servoCaching;
            pulseSame &= p.pulsed == p0.pulsed && p.pulseBandDeg == p0.pulseBandDeg
                    && p.pulseTolDeg == p0.pulseTolDeg && p.pulsePower == p0.pulsePower
                    && p.pulseMs == p0.pulseMs
                    && p.pulseStationaryDegPerSec == p0.pulseStationaryDegPerSec
                    && p.pulseCoastMs == p0.pulseCoastMs;
        }
        if (cacheSame) {
            decl(sb, "double", "turnServoCaching", num(p0.servoCaching, 2));
        } else {
            warnings.add("servo caching differs between pods; Constants.turnServoCaching is one "
                    + "value for all four. Not exported.");
        }
        sb.append("\n// Pulsed final approach, shared by all four pods (see buildPod)\n");
        if (pulseSame && p0.pulsed) {
            decl(sb, "double", "pulseBandDeg", num(p0.pulseBandDeg, 1));
            decl(sb, "double", "pulseToleranceDeg", num(p0.pulseTolDeg, 1));
            decl(sb, "double", "pulsePower", num(p0.pulsePower, 3));
            decl(sb, "double", "pulseSeconds", num(p0.pulseMs / 1000.0, 3));
            decl(sb, "double", "pulseStationaryDegPerSec", num(p0.pulseStationaryDegPerSec, 1));
            decl(sb, "double", "pulseCoastSeconds", num(p0.pulseCoastMs / 1000.0, 2));
        } else if (!pulseSame) {
            warnings.add("pulsed-approach settings differ between pods; Constants.java holds one "
                    + "set for all four. Not exported.");
        } else {
            warnings.add("pulsed approach is OFF in the tool; Constants.buildPod always enables "
                    + "it. Not exported.");
        }

        if (!warnings.isEmpty()) {
            sb.append("\n// WARNINGS - tuned in the tool, NOT expressible in Constants.java:\n");
            for (String w : warnings) {
                sb.append("//   ").append(w).append('\n');
            }
        }
        return sb.toString();
    }

    private static void decl(StringBuilder sb, String type, String name, String value) {
        sb.append("public static final ").append(type).append(' ').append(name).append(" = ")
                .append(value).append(";\n");
    }

    private static String array(String[] values) {
        StringBuilder sb = new StringBuilder("{");
        for (int i = 0; i < values.length; i++) {
            sb.append(i == 0 ? "" : ", ").append(values[i]);
        }
        return sb.append('}').toString();
    }

    /**
     * A plain decimal with at least {@code minDecimals} places and no trailing noise beyond what
     * the value carries (to 6 places), so an untouched value prints exactly as Constants.java
     * already spells it and the splice diff shows only what actually moved.
     */
    static String num(double v, int minDecimals) {
        BigDecimal b = BigDecimal.valueOf(v).setScale(6, RoundingMode.HALF_UP)
                .stripTrailingZeros();
        if (b.scale() < minDecimals) {
            b = b.setScale(minDecimals, RoundingMode.UNNECESSARY);
        }
        return b.toPlainString();
    }
}
