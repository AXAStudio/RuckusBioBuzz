package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
public class PollenDetectionPipeline implements VisionProcessor {
    public static final String DEFAULT_WEBCAM_NAME = "Webcam 1";
    public static final Size DEFAULT_CAMERA_RESOLUTION = new Size(640, 480);

    private static final double ROI_TOP_FRACTION = 0.28;

    private static final Scalar HSV_STRICT_LOW = new Scalar(15, 192, 211);
    private static final Scalar HSV_STRICT_HIGH = new Scalar(26, 255, 255);
    private static final Scalar HSV_WIDE_LOW = new Scalar(12, 163, 183);
    private static final Scalar HSV_WIDE_HIGH = new Scalar(29, 255, 255);

    private static final int RGB_MIN_R = 195;
    private static final int RGB_MIN_G = 126;
    private static final int RGB_MAX_B = 83;
    private static final int YELLOW_MARGIN = 120;
    private static final int MIN_YELLOW_SCORE = 110;

    private static final double MIN_AREA_PX = 260.0;
    private static final double MAX_AREA_FRACTION = 0.36;
    private static final double MAX_ASPECT = 5.0;
    private static final double MIN_EXTENT = 0.22;
    private static final double MIN_FILL_RATIO = 0.16;
    private static final double MIN_CIRCULARITY = 0.26;
    private static final double MIN_CONFIDENCE = 0.56;

    private static final int OPEN_KERNEL_PX = 3;
    private static final int CLOSE_KERNEL_PX = 15;
    private static final int MAX_CONTOURS = 24;

    private static final double CLUMP_MAX_ASPECT = 24.0;
    private static final double CLUMP_PEAK_THRESHOLD_FRACTION = 0.38;
    private static final double CLUMP_PEAK_MIN_RADIUS_PX = 4.0;
    private static final double CLUMP_PEAK_MIN_DISTANCE_RADIUS = 1.05;
    private static final double CLUMP_AREA_FILL_ESTIMATE = 0.68;
    private static final double CLUMP_WIDTH_SPACING_RADIUS = 1.55;
    private static final double CLUMP_FRAGMENT_RADIUS_FRACTION = 0.22;
    private static final double CLUMP_FRAGMENT_MIN_MINOR_PX = 42.0;
    private static final double CIRCLE_HOUGH_PARAM2 = 20.0;
    private static final double CIRCLE_MIN_SCORE = 0.70;
    private static final double CIRCLE_MIN_YELLOW_FRACTION = 0.26;
    private static final int CIRCLE_MIN_RADIUS_PX = 8;
    private static final int CIRCLE_MAX_RADIUS_PX = 72;

    private static final Comparator<Detection> LARGEST_CLUMP_ORDER =
            Comparator.comparingInt((Detection detection) -> detection.estimatedCount)
                    .thenComparingDouble(detection -> detection.areaPx)
                    .thenComparingDouble(detection -> detection.confidence)
                    .reversed();

    private final Object lock = new Object();
    private final Telemetry telemetry;
    private final List<Detection> detections = new ArrayList<>();

    private final Mat rgbRoi = new Mat();
    private final Mat hsv = new Mat();
    private final Mat strictMask = new Mat();
    private final Mat wideMask = new Mat();
    private final Mat rgbYellowMask = new Mat();
    private final Mat mask = new Mat();
    private final Mat contourMask = new Mat();
    private final Mat openKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE,
            new org.opencv.core.Size(oddKernelSize(OPEN_KERNEL_PX, 1), oddKernelSize(OPEN_KERNEL_PX, 1))
    );
    private final Mat closeKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE,
            new org.opencv.core.Size(oddKernelSize(CLOSE_KERNEL_PX, 3), oddKernelSize(CLOSE_KERNEL_PX, 3))
    );

    private int frameWidth = 640;
    private int frameHeight = 480;
    private int roiTop = computeRoiTop(480);
    private Detection best = null;
    private double steeringError = 0.0;
    private int maskPixels = 0;

    private final Paint boxPaint = new Paint();
    private final Paint textPaint = new Paint();
    private final Paint textBackgroundPaint = new Paint();
    private final Paint guidePaint = new Paint();
    private final Paint memberPaint = new Paint();
    private final Paint arrowPaint = new Paint();

    public PollenDetectionPipeline() {
        this(null);
    }

    public PollenDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(5.0f);
        boxPaint.setColor(Color.rgb(80, 255, 90));

        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setTextSize(32.0f);
        textPaint.setFakeBoldText(true);
        textPaint.setColor(Color.rgb(80, 255, 90));

        textBackgroundPaint.setStyle(Paint.Style.FILL);
        textBackgroundPaint.setColor(Color.argb(190, 0, 0, 0));

        guidePaint.setStyle(Paint.Style.STROKE);
        guidePaint.setStrokeWidth(2.0f);
        guidePaint.setColor(Color.argb(190, 255, 255, 255));

        memberPaint.setStyle(Paint.Style.STROKE);
        memberPaint.setStrokeWidth(2.0f);
        memberPaint.setColor(Color.WHITE);

        arrowPaint.setStyle(Paint.Style.STROKE);
        arrowPaint.setStrokeWidth(4.0f);
        arrowPaint.setColor(Color.rgb(255, 150, 40));
    }

    public VisionPortal buildVisionPortal(HardwareMap hardwareMap) {
        return buildVisionPortal(hardwareMap, DEFAULT_WEBCAM_NAME);
    }

    public VisionPortal buildVisionPortal(HardwareMap hardwareMap, String webcamName) {
        return new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(DEFAULT_CAMERA_RESOLUTION)
                .addProcessor(this)
                .build();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        synchronized (lock) {
            frameWidth = width;
            frameHeight = height;
            roiTop = computeRoiTop(height);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int width = frame.cols();
        int height = frame.rows();
        int top = computeRoiTop(height);
        int roiHeight = Math.max(1, height - top);
        Rect roiRect = new Rect(0, top, width, roiHeight);

        Mat roi = frame.submat(roiRect);
        roi.copyTo(rgbRoi);
        roi.release();

        buildMask(rgbRoi, hsv, mask);
        int nextMaskPixels = Core.countNonZero(mask);
        mask.copyTo(contourMask);

        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(contourMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        if (contours.size() > MAX_CONTOURS) {
            Collections.sort(contours, (left, right) -> Double.compare(Imgproc.contourArea(right), Imgproc.contourArea(left)));
            List<MatOfPoint> retained = new ArrayList<>(contours.subList(0, MAX_CONTOURS));
            for (int i = MAX_CONTOURS; i < contours.size(); i++) {
                contours.get(i).release();
            }
            contours = retained;
        }

        double maxArea = Math.max(MIN_AREA_PX, width * height * MAX_AREA_FRACTION);
        List<Detection> nextDetections = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Detection detection = candidateFromContour(rgbRoi, hsv, mask, contour, width, top, maxArea);
            if (detection == null) {
                detection = candidateFromClumpContour(rgbRoi, hsv, mask, contour, width, top, maxArea);
            }
            if (detection != null) {
                nextDetections.add(detection);
            }
            contour.release();
        }
        if (hasFragmentedCandidate(nextDetections)) {
            refineFragmentedCandidatesWithCircles(
                    nextDetections,
                    detectVisibleBallCircles(rgbRoi, hsv, mask, top),
                    width
            );
        }
        Collections.sort(nextDetections, LARGEST_CLUMP_ORDER);

        Detection largestClump = largestClump(nextDetections);
        Detection nextBest = largestClump != null ? largestClump : (nextDetections.isEmpty() ? null : nextDetections.get(0));

        synchronized (lock) {
            frameWidth = width;
            frameHeight = height;
            roiTop = top;
            maskPixels = nextMaskPixels;
            detections.clear();
            detections.addAll(nextDetections);
            best = nextBest;
            steeringError = best == null ? 0.0 : best.steeringError;
        }

        if (telemetry != null) {
            telemetry.addData("[Pollen] detections", nextDetections.size());
            telemetry.addData("[Pollen] mask px", nextMaskPixels);
            telemetry.addData("[Pollen] largest", nextBest == null ? "none" : nextBest);
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Detection target;
        int localFrameWidth;
        int localFrameHeight;
        int localRoiTop;
        synchronized (lock) {
            target = best;
            localFrameWidth = frameWidth;
            localFrameHeight = frameHeight;
            localRoiTop = roiTop;
        }

        float xScale = onscreenWidth / (float) Math.max(1, localFrameWidth);
        float yScale = onscreenHeight / (float) Math.max(1, localFrameHeight);
        float centerX = onscreenWidth * 0.5f;

        canvas.drawLine(centerX, 0, centerX, onscreenHeight, guidePaint);
        canvas.drawLine(0, localRoiTop * yScale, onscreenWidth, localRoiTop * yScale, guidePaint);

        if (target == null) {
            drawLabel(canvas, "NO POLLEN CLUMP", 8.0f, 40.0f, Color.RED);
            return;
        }

        RectF box = new RectF(
                target.boundX * xScale,
                target.boundY * yScale,
                (target.boundX + target.boundW) * xScale,
                (target.boundY + target.boundH) * yScale
        );
        canvas.drawRect(box, boxPaint);

        float targetX = (float) target.centerX * xScale;
        float targetY = (float) target.centerY * yScale;
        canvas.drawCircle(targetX, targetY, Math.max(7.0f, (float) target.radiusPx * xScale), boxPaint);
        canvas.drawLine(targetX - 14.0f, targetY, targetX + 14.0f, targetY, boxPaint);
        canvas.drawLine(targetX, targetY - 14.0f, targetX, targetY + 14.0f, boxPaint);

        for (int i = 0; i < Math.min(40, target.memberCenters.size()); i++) {
            Center member = target.memberCenters.get(i);
            canvas.drawCircle((float) member.x * xScale, (float) member.y * yScale, 5.0f, memberPaint);
        }

        float arrowY = onscreenHeight - 28.0f;
        canvas.drawLine(centerX, arrowY, targetX, arrowY, arrowPaint);
        float arrowDir = targetX >= centerX ? -1.0f : 1.0f;
        canvas.drawLine(targetX, arrowY, targetX + arrowDir * 14.0f, arrowY - 8.0f, arrowPaint);
        canvas.drawLine(targetX, arrowY, targetX + arrowDir * 14.0f, arrowY + 8.0f, arrowPaint);

        String label = String.format(
                Locale.US,
                "LARGEST clump=%d %s err=%+.2f",
                target.estimatedCount,
                target.direction().toUpperCase(Locale.US),
                target.steeringError
        );
        drawLabel(canvas, label, Math.max(4.0f, box.left), Math.max(40.0f, box.top - 8.0f), Color.rgb(80, 255, 90));
    }

    public boolean hasTarget() {
        synchronized (lock) {
            return best != null;
        }
    }

    public Detection getBestDetection() {
        synchronized (lock) {
            return best;
        }
    }

    public List<Detection> getDetections() {
        synchronized (lock) {
            return new ArrayList<>(detections);
        }
    }

    public double getSteeringError() {
        synchronized (lock) {
            return steeringError;
        }
    }

    public int getMaskPixels() {
        synchronized (lock) {
            return maskPixels;
        }
    }

    private void buildMask(Mat rgb, Mat hsvOut, Mat maskOut) {
        Imgproc.cvtColor(rgb, hsvOut, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvOut, HSV_STRICT_LOW, HSV_STRICT_HIGH, strictMask);
        Core.inRange(hsvOut, HSV_WIDE_LOW, HSV_WIDE_HIGH, wideMask);

        rgbYellowMask.create(rgb.rows(), rgb.cols(), CvType.CV_8UC1);
        byte[] rgbBytes = new byte[(int) (rgb.total() * rgb.channels())];
        byte[] hsvBytes = new byte[(int) (hsvOut.total() * hsvOut.channels())];
        byte[] yellowBytes = new byte[(int) rgb.total()];
        rgb.get(0, 0, rgbBytes);
        hsvOut.get(0, 0, hsvBytes);

        for (int pixel = 0; pixel < yellowBytes.length; pixel++) {
            int rgbIndex = pixel * 3;
            int hsvIndex = pixel * 3;
            int r = rgbBytes[rgbIndex] & 0xFF;
            int g = rgbBytes[rgbIndex + 1] & 0xFF;
            int b = rgbBytes[rgbIndex + 2] & 0xFF;
            int saturation = hsvBytes[hsvIndex + 1] & 0xFF;
            double score = yellowScore(r, g, b, saturation);
            boolean yellow =
                    r >= RGB_MIN_R
                            && g >= RGB_MIN_G
                            && b <= RGB_MAX_B
                            && Math.min(r, g) - b >= YELLOW_MARGIN
                            && score >= MIN_YELLOW_SCORE;
            yellowBytes[pixel] = yellow ? (byte) 255 : 0;
        }
        rgbYellowMask.put(0, 0, yellowBytes);

        Core.bitwise_and(wideMask, rgbYellowMask, maskOut);
        Core.bitwise_or(strictMask, maskOut, maskOut);
        Imgproc.medianBlur(maskOut, maskOut, 3);
        Imgproc.morphologyEx(maskOut, maskOut, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(maskOut, maskOut, Imgproc.MORPH_CLOSE, closeKernel);
    }

    private Detection candidateFromContour(
            Mat rgb,
            Mat hsvMat,
            Mat cleanedMask,
            MatOfPoint contour,
            int fullFrameWidth,
            int top,
            double maxArea
    ) {
        double area = Imgproc.contourArea(contour);
        if (area < MIN_AREA_PX || area > maxArea) {
            return null;
        }

        Rect rect = Imgproc.boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0 || rect.y <= 1) {
            return null;
        }

        double aspect = rect.width / (double) rect.height;
        if (aspect > MAX_ASPECT || aspect < 1.0 / MAX_ASPECT) {
            return null;
        }

        double perimeter = contourPerimeter(contour);
        double circularity = perimeter <= 0.0 ? 0.0 : 4.0 * Math.PI * area / (perimeter * perimeter);
        double extent = area / (rect.width * (double) rect.height);

        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);
        Point circleCenter = new Point();
        float[] radiusHolder = new float[1];
        Imgproc.minEnclosingCircle(contour2f, circleCenter, radiusHolder);
        contour2f.release();
        double radius = radiusHolder[0];
        double circleArea = radius > 0.0 ? Math.PI * radius * radius : 1.0;
        double fillRatio = area / circleArea;

        if (extent < MIN_EXTENT || fillRatio < MIN_FILL_RATIO) {
            return null;
        }
        if (circularity < MIN_CIRCULARITY && aspect < 1.55) {
            return null;
        }

        Mat contourMask = contourMask(contour, rect);
        Mat cleanedRoi = cleanedMask.submat(rect);
        Mat covered = new Mat();
        Core.bitwise_and(cleanedRoi, contourMask, covered);
        int contourPixels = Math.max(1, Core.countNonZero(contourMask));
        double maskCoverage = Core.countNonZero(covered) / (double) contourPixels;
        cleanedRoi.release();
        covered.release();
        if (maskCoverage < 0.22) {
            contourMask.release();
            return null;
        }

        Mat rgbPatch = rgb.submat(rect);
        Mat hsvPatch = hsvMat.submat(rect);
        Scalar meanRgb = Core.mean(rgbPatch, contourMask);
        Scalar meanHsv = Core.mean(hsvPatch, contourMask);
        rgbPatch.release();
        hsvPatch.release();
        contourMask.release();

        Double colorConfidence = colorConfidence(meanRgb, meanHsv);
        if (colorConfidence == null) {
            return null;
        }

        double circularityScore = clamp((circularity - 0.20) / 0.62, 0.0, 1.0);
        double aspectScore = 1.0 - clamp(Math.abs(Math.log(Math.max(0.01, aspect))) / Math.log(MAX_ASPECT), 0.0, 1.0);
        double fillScore = clamp((fillRatio - 0.15) / 0.55, 0.0, 1.0);
        double extentScore = clamp((extent - MIN_EXTENT) / 0.42, 0.0, 1.0);
        double coverageScore = clamp((maskCoverage - 0.22) / 0.58, 0.0, 1.0);
        double shapeConfidence =
                0.34 * circularityScore
                        + 0.24 * aspectScore
                        + 0.22 * fillScore
                        + 0.20 * extentScore;
        double confidence = 0.58 * colorConfidence + 0.30 * shapeConfidence + 0.12 * coverageScore;
        if (confidence < MIN_CONFIDENCE) {
            return null;
        }

        Moments moments = Imgproc.moments(contour);
        double centerX = Math.abs(moments.m00) > 1e-6 ? moments.m10 / moments.m00 : circleCenter.x;
        double centerYInRoi = Math.abs(moments.m00) > 1e-6 ? moments.m01 / moments.m00 : circleCenter.y;

        MemberEstimate estimate = estimateClumpMembers(cleanedMask, contour, rect, area, top);
        if (estimate.estimatedCount > 1 && !estimate.centers.isEmpty()) {
            centerX = meanX(estimate.centers);
            centerYInRoi = meanY(estimate.centers) - top;
        }

        double fullCenterY = centerYInRoi + top;
        double steering = (centerX - fullFrameWidth * 0.5) / (fullFrameWidth * 0.5);
        return new Detection(
                centerX,
                fullCenterY,
                area,
                rect.x,
                rect.y + top,
                rect.width,
                rect.height,
                estimate.estimatedCount,
                steering,
                confidence,
                fillRatio,
                aspect,
                estimate.estimatedCount > 1 ? estimate.radiusPx : radius,
                circularity,
                extent,
                maskCoverage,
                estimate.countConfidence,
                estimate.centers,
                estimate.estimatedCount > 1
        );
    }

    private Detection candidateFromClumpContour(
            Mat rgb,
            Mat hsvMat,
            Mat cleanedMask,
            MatOfPoint contour,
            int fullFrameWidth,
            int top,
            double maxArea
    ) {
        double area = Imgproc.contourArea(contour);
        if (area < MIN_AREA_PX * 1.35 || area > maxArea) {
            return null;
        }

        Rect rect = Imgproc.boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0 || rect.y <= 1) {
            return null;
        }

        double aspect = rect.width / (double) rect.height;
        double inverseAspect = rect.height / (double) rect.width;
        if (aspect > CLUMP_MAX_ASPECT || inverseAspect > CLUMP_MAX_ASPECT) {
            return null;
        }

        double extent = area / (rect.width * (double) rect.height);
        if (extent < Math.max(0.10, MIN_EXTENT * 0.45)) {
            return null;
        }

        double perimeter = contourPerimeter(contour);
        double circularity = perimeter <= 0.0 ? 0.0 : 4.0 * Math.PI * area / (perimeter * perimeter);

        Mat contourMask = contourMask(contour, rect);
        Mat cleanedRoi = cleanedMask.submat(rect);
        Mat covered = new Mat();
        Core.bitwise_and(cleanedRoi, contourMask, covered);
        int contourPixels = Math.max(1, Core.countNonZero(contourMask));
        double maskCoverage = Core.countNonZero(covered) / (double) contourPixels;
        cleanedRoi.release();
        covered.release();
        if (maskCoverage < 0.34) {
            contourMask.release();
            return null;
        }

        Mat rgbPatch = rgb.submat(rect);
        Mat hsvPatch = hsvMat.submat(rect);
        Scalar meanRgb = Core.mean(rgbPatch, contourMask);
        Scalar meanHsv = Core.mean(hsvPatch, contourMask);
        rgbPatch.release();
        hsvPatch.release();
        contourMask.release();

        Double colorConfidence = colorConfidence(meanRgb, meanHsv);
        if (colorConfidence == null || colorConfidence < MIN_CONFIDENCE * 0.58) {
            return null;
        }

        MemberEstimate estimate = estimateClumpMembers(cleanedMask, contour, rect, area, top);
        if (estimate.estimatedCount <= 1) {
            return null;
        }

        double centerX;
        double centerY;
        if (!estimate.centers.isEmpty()) {
            centerX = meanX(estimate.centers);
            centerY = meanY(estimate.centers);
        } else {
            Moments moments = Imgproc.moments(contour);
            if (Math.abs(moments.m00) > 1e-6) {
                centerX = moments.m10 / moments.m00;
                centerY = moments.m01 / moments.m00 + top;
            } else {
                centerX = rect.x + rect.width * 0.5;
                centerY = rect.y + rect.height * 0.5 + top;
            }
        }

        double equivalentRadius = Math.sqrt(area / Math.max(1.0, Math.PI * estimate.estimatedCount));
        double fillRatio = clamp(equivalentRadius / Math.max(1.0, estimate.radiusPx), 0.0, 1.4);
        double coverageScore = clamp((maskCoverage - 0.34) / 0.54, 0.0, 1.0);
        double extentScore = clamp((extent - 0.10) / 0.46, 0.0, 1.0);
        double confidence = 0.56 * colorConfidence + 0.18 * coverageScore + 0.12 * extentScore + 0.14 * estimate.countConfidence;
        if (confidence < MIN_CONFIDENCE * 0.78) {
            return null;
        }

        double steering = (centerX - fullFrameWidth * 0.5) / (fullFrameWidth * 0.5);
        return new Detection(
                centerX,
                centerY,
                area,
                rect.x,
                rect.y + top,
                rect.width,
                rect.height,
                estimate.estimatedCount,
                steering,
                confidence,
                fillRatio,
                aspect,
                estimate.radiusPx,
                circularity,
                extent,
                maskCoverage,
                estimate.countConfidence,
                estimate.centers,
                true
        );
    }

    private List<PollenBall> detectVisibleBallCircles(Mat rgb, Mat hsvMat, Mat pollenMask, int top) {
        Mat value = new Mat();
        Mat saturation = new Mat();
        Mat valueEqualized = new Mat();
        Mat saturationEqualized = new Mat();
        Mat circleImage = new Mat();
        Mat edges = new Mat();
        Mat circles = new Mat();

        Core.extractChannel(hsvMat, value, 2);
        Core.extractChannel(hsvMat, saturation, 1);
        Imgproc.equalizeHist(value, valueEqualized);
        Imgproc.equalizeHist(saturation, saturationEqualized);
        Core.addWeighted(valueEqualized, 0.55, saturationEqualized, 0.45, 0.0, circleImage);
        Imgproc.GaussianBlur(circleImage, circleImage, new org.opencv.core.Size(5, 5), 1.1);
        Imgproc.Canny(circleImage, edges, 70, 150);

        Imgproc.HoughCircles(
                circleImage,
                circles,
                Imgproc.HOUGH_GRADIENT,
                1.2,
                12.0,
                90.0,
                CIRCLE_HOUGH_PARAM2,
                CIRCLE_MIN_RADIUS_PX,
                CIRCLE_MAX_RADIUS_PX
        );

        List<PollenBall> result = new ArrayList<>();
        if (circles.empty()) {
            value.release();
            saturation.release();
            valueEqualized.release();
            saturationEqualized.release();
            circleImage.release();
            edges.release();
            circles.release();
            return result;
        }

        int rows = rgb.rows();
        int cols = rgb.cols();
        byte[] rgbBytes = new byte[(int) (rgb.total() * rgb.channels())];
        byte[] hsvBytes = new byte[(int) (hsvMat.total() * hsvMat.channels())];
        byte[] maskBytes = new byte[(int) pollenMask.total()];
        byte[] edgeBytes = new byte[(int) edges.total()];
        rgb.get(0, 0, rgbBytes);
        hsvMat.get(0, 0, hsvBytes);
        pollenMask.get(0, 0, maskBytes);
        edges.get(0, 0, edgeBytes);

        float[] circleData = new float[(int) (circles.total() * circles.channels())];
        circles.get(0, 0, circleData);
        List<PollenBall> scored = new ArrayList<>();
        for (int i = 0; i + 2 < circleData.length; i += 3) {
            double circleX = circleData[i];
            double circleY = circleData[i + 1];
            double radius = circleData[i + 2];
            if (circleY <= 2.0) {
                continue;
            }

            int minX = Math.max(0, (int) Math.floor(circleX - radius * 1.08));
            int maxX = Math.min(cols - 1, (int) Math.ceil(circleX + radius * 1.08));
            int minY = Math.max(0, (int) Math.floor(circleY - radius * 1.08));
            int maxY = Math.min(rows - 1, (int) Math.ceil(circleY + radius * 1.08));
            int diskPixels = 0;
            int ringPixels = 0;
            int yellowPixels = 0;
            int strictPixels = 0;
            int edgePixels = 0;
            double sumH = 0.0;
            double sumS = 0.0;
            double sumV = 0.0;
            double radiusSq = radius * radius;
            double outerRingSq = (radius * 1.07) * (radius * 1.07);
            double innerRingSq = (radius * 0.82) * (radius * 0.82);

            for (int y = minY; y <= maxY; y++) {
                double dy = y - circleY;
                for (int x = minX; x <= maxX; x++) {
                    double dx = x - circleX;
                    double distanceSq = dx * dx + dy * dy;
                    int index = y * cols + x;
                    int rgbIndex = index * 3;
                    int hsvIndex = index * 3;
                    int r = rgbBytes[rgbIndex] & 0xFF;
                    int g = rgbBytes[rgbIndex + 1] & 0xFF;
                    int b = rgbBytes[rgbIndex + 2] & 0xFF;
                    int hue = hsvBytes[hsvIndex] & 0xFF;
                    int sat = hsvBytes[hsvIndex + 1] & 0xFF;
                    int val = hsvBytes[hsvIndex + 2] & 0xFF;

                    if (distanceSq <= radiusSq) {
                        diskPixels++;
                        sumH += hue;
                        sumS += sat;
                        sumV += val;
                        boolean yellowish =
                                hue >= 10
                                        && hue <= 45
                                        && sat >= 45
                                        && val >= 55
                                        && Math.min(r, g) - b > 12;
                        if (yellowish) {
                            yellowPixels++;
                        }
                        if ((maskBytes[index] & 0xFF) > 0) {
                            strictPixels++;
                        }
                    }
                    if (distanceSq <= outerRingSq && distanceSq >= innerRingSq) {
                        ringPixels++;
                        if ((edgeBytes[index] & 0xFF) > 0) {
                            edgePixels++;
                        }
                    }
                }
            }

            if (diskPixels <= 0 || ringPixels <= 0) {
                continue;
            }

            double yellowFraction = yellowPixels / (double) diskPixels;
            double strictFraction = strictPixels / (double) diskPixels;
            double edgeFraction = edgePixels / (double) ringPixels;
            double meanH = sumH / diskPixels;
            double meanS = sumS / diskPixels;
            double meanV = sumV / diskPixels;
            if (yellowFraction < CIRCLE_MIN_YELLOW_FRACTION || meanS < 70.0 || meanV < 65.0) {
                continue;
            }
            if (radius > 28.0 && (meanH > 38.0 || yellowFraction < 0.72)) {
                continue;
            }

            double hueScore = Math.max(0.0, 1.0 - Math.abs(meanH - 25.0) / 22.0);
            double score =
                    0.42 * Math.min(yellowFraction / 0.55, 1.0)
                            + 0.18 * Math.min(strictFraction / 0.24, 1.0)
                            + 0.22 * Math.min(edgeFraction / 0.08, 1.0)
                            + 0.18 * hueScore;
            if (score >= CIRCLE_MIN_SCORE) {
                scored.add(new PollenBall(circleX, circleY + top, radius, score));
            }
        }

        Collections.sort(scored, (left, right) -> Double.compare(right.confidence, left.confidence));
        for (PollenBall ball : scored) {
            boolean overlapsExisting = false;
            for (PollenBall other : result) {
                double distance = Math.hypot(ball.centerX - other.centerX, ball.centerY - other.centerY);
                if (distance < Math.max(12.0, Math.min(ball.radiusPx, other.radiusPx) * 0.78)
                        || distance < (ball.radiusPx + other.radiusPx) * 0.42) {
                    overlapsExisting = true;
                    break;
                }
            }
            if (!overlapsExisting) {
                result.add(ball);
            }
        }

        value.release();
        saturation.release();
        valueEqualized.release();
        saturationEqualized.release();
        circleImage.release();
        edges.release();
        circles.release();
        return result;
    }

    private void refineFragmentedCandidatesWithCircles(List<Detection> candidates, List<PollenBall> balls, int fullFrameWidth) {
        if (balls.isEmpty()) {
            return;
        }

        Collections.sort(balls, (left, right) -> Double.compare(right.confidence, left.confidence));
        List<List<PollenBall>> assignments = new ArrayList<>();
        for (int i = 0; i < candidates.size(); i++) {
            assignments.add(new ArrayList<>());
        }

        for (PollenBall ball : balls) {
            int bestIndex = -1;
            double bestArea = Double.POSITIVE_INFINITY;
            for (int i = 0; i < candidates.size(); i++) {
                Detection candidate = candidates.get(i);
                boolean inside =
                        candidate.boundX <= ball.centerX
                                && ball.centerX <= candidate.boundX + candidate.boundW
                                && candidate.boundY <= ball.centerY
                                && ball.centerY <= candidate.boundY + candidate.boundH;
                if (!inside) {
                    continue;
                }
                double boxArea = candidate.boundW * (double) candidate.boundH;
                if (boxArea < bestArea) {
                    bestArea = boxArea;
                    bestIndex = i;
                }
            }
            if (bestIndex >= 0) {
                assignments.get(bestIndex).add(ball);
            }
        }

        for (int i = 0; i < candidates.size(); i++) {
            List<PollenBall> assigned = assignments.get(i);
            if (assigned.isEmpty()) {
                continue;
            }

            Detection candidate = candidates.get(i);
            double aspect = Math.max(candidate.boundW, candidate.boundH) / (double) Math.max(1, Math.min(candidate.boundW, candidate.boundH));
            boolean shouldOverride = candidateIsFragmented(candidate)
                    || (aspect <= 2.4 && candidate.estimatedCount > assigned.size());
            if (!shouldOverride) {
                continue;
            }

            List<Center> centers = new ArrayList<>();
            double centerX = 0.0;
            double centerY = 0.0;
            double minConfidence = 1.0;
            List<Double> radii = new ArrayList<>();
            for (PollenBall ball : assigned) {
                centers.add(new Center(ball.centerX, ball.centerY));
                centerX += ball.centerX;
                centerY += ball.centerY;
                minConfidence = Math.min(minConfidence, ball.confidence);
                radii.add(ball.radiusPx);
            }
            centerX /= assigned.size();
            centerY /= assigned.size();
            Collections.sort(radii);
            double radius = radii.get(radii.size() / 2);
            if (radii.size() % 2 == 0) {
                radius = (radii.get(radii.size() / 2 - 1) + radii.get(radii.size() / 2)) * 0.5;
            }

            candidates.set(i, new Detection(
                    centerX,
                    centerY,
                    candidate.areaPx,
                    candidate.boundX,
                    candidate.boundY,
                    candidate.boundW,
                    candidate.boundH,
                    assigned.size(),
                    (centerX - fullFrameWidth * 0.5) / (fullFrameWidth * 0.5),
                    candidate.confidence,
                    candidate.fillRatio,
                    candidate.aspectRatio,
                    radius,
                    candidate.circularity,
                    candidate.extent,
                    candidate.maskCoverage,
                    Math.max(candidate.countConfidence, minConfidence),
                    centers,
                    assigned.size() > 1
            ));
        }
    }

    private boolean hasFragmentedCandidate(List<Detection> candidates) {
        for (Detection candidate : candidates) {
            if (candidateIsFragmented(candidate)) {
                return true;
            }
        }
        return false;
    }

    private boolean candidateIsFragmented(Detection candidate) {
        double minorAxis = Math.max(1.0, Math.min(candidate.boundW, candidate.boundH));
        return candidate.estimatedCount > 1
                && minorAxis >= CLUMP_FRAGMENT_MIN_MINOR_PX
                && candidate.radiusPx / minorAxis < CLUMP_FRAGMENT_RADIUS_FRACTION;
    }

    private MemberEstimate estimateClumpMembers(Mat cleanedMask, MatOfPoint contour, Rect rect, double area, int top) {
        Mat roiMask = contourMask(contour, rect);
        Mat distance = new Mat();
        Imgproc.distanceTransform(roiMask, distance, Imgproc.DIST_L2, 5);

        Core.MinMaxLocResult minMax = Core.minMaxLoc(distance);
        double maxRadius = minMax.maxVal;
        if (maxRadius < CLUMP_PEAK_MIN_RADIUS_PX) {
            roiMask.release();
            distance.release();
            return new MemberEstimate(new ArrayList<>(), 1, 0.25, Math.max(1.0, Math.min(rect.width, rect.height) * 0.5));
        }

        Mat dilated = new Mat();
        Imgproc.dilate(distance, dilated, Mat.ones(5, 5, CvType.CV_8U));

        float[] distanceData = new float[(int) distance.total()];
        float[] dilatedData = new float[(int) dilated.total()];
        byte[] maskData = new byte[(int) roiMask.total()];
        distance.get(0, 0, distanceData);
        dilated.get(0, 0, dilatedData);
        roiMask.get(0, 0, maskData);

        double threshold = Math.max(CLUMP_PEAK_MIN_RADIUS_PX, maxRadius * CLUMP_PEAK_THRESHOLD_FRACTION);
        Mat localMaxMask = Mat.zeros(rect.height, rect.width, CvType.CV_8UC1);
        byte[] localMaxData = new byte[(int) roiMask.total()];
        int width = rect.width;
        for (int index = 0; index < distanceData.length; index++) {
            double radius = distanceData[index];
            if ((maskData[index] & 0xFF) == 0 || radius < threshold || radius < dilatedData[index] - 1e-5) {
                continue;
            }
            localMaxData[index] = (byte) 255;
        }
        localMaxMask.put(0, 0, localMaxData);

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int componentCount = Imgproc.connectedComponentsWithStats(localMaxMask, labels, stats, centroids, 8, CvType.CV_32S);
        int[] labelData = new int[(int) labels.total()];
        double[] centroidData = new double[(int) (centroids.total() * centroids.channels())];
        labels.get(0, 0, labelData);
        centroids.get(0, 0, centroidData);

        List<Peak> peaks = new ArrayList<>();
        for (int label = 1; label < componentCount; label++) {
            double bestRadius = 0.0;
            for (int index = 0; index < labelData.length; index++) {
                if (labelData[index] != label) {
                    continue;
                }
                bestRadius = Math.max(bestRadius, distanceData[index]);
            }
            double peakX = centroidData[label * 2];
            double peakY = centroidData[label * 2 + 1];
            peaks.add(new Peak(bestRadius, rect.x + peakX, rect.y + peakY + top));
        }
        Collections.sort(peaks, (left, right) -> Double.compare(right.radius, left.radius));

        double radiusEstimate = medianRadius(peaks, maxRadius);
        radiusEstimate = Math.max(CLUMP_PEAK_MIN_RADIUS_PX, radiusEstimate);
        double minPeakDistance = Math.max(3.0, radiusEstimate * CLUMP_PEAK_MIN_DISTANCE_RADIUS);

        List<Peak> selected = new ArrayList<>();
        for (Peak peak : peaks) {
            boolean farEnough = true;
            for (Peak other : selected) {
                if (Math.hypot(peak.x - other.x, peak.y - other.y) < minPeakDistance) {
                    farEnough = false;
                    break;
                }
            }
            if (farEnough) {
                selected.add(peak);
            }
        }

        int peakCount = selected.size();
        double singleArea = Math.PI * radiusEstimate * radiusEstimate * CLUMP_AREA_FILL_ESTIMATE;
        double areaEstimate = area / Math.max(1.0, singleArea);
        double majorAxis = Math.max(rect.width, rect.height);
        double minorAxis = Math.max(1.0, Math.min(rect.width, rect.height));
        double lineEstimate = majorAxis / Math.max(1.0, radiusEstimate * CLUMP_WIDTH_SPACING_RADIUS);

        List<Double> estimates = new ArrayList<>();
        if (peakCount > 0) {
            estimates.add((double) peakCount);
        }
        if (areaEstimate >= 0.75) {
            estimates.add(areaEstimate);
        }
        if (majorAxis / minorAxis > 1.35) {
            estimates.add(lineEstimate);
        }
        if (estimates.isEmpty()) {
            estimates.add(1.0);
        }
        Collections.sort(estimates);

        double rawCount;
        int middle = estimates.size() / 2;
        if (estimates.size() % 2 == 1) {
            rawCount = estimates.get(middle);
        } else {
            rawCount = (estimates.get(middle - 1) + estimates.get(middle)) * 0.5;
        }
        int estimatedCount = Math.max(1, (int) Math.round(rawCount));

        if (estimatedCount > 1 && peakCount <= 1 && majorAxis / minorAxis > 1.35) {
            estimatedCount = Math.max(estimatedCount, (int) Math.round(lineEstimate));
        }

        boolean usedFragmentGuard = false;
        double radiusFraction = radiusEstimate / Math.max(1.0, minorAxis);
        if (estimatedCount > 1 && minorAxis >= CLUMP_FRAGMENT_MIN_MINOR_PX && radiusFraction < CLUMP_FRAGMENT_RADIUS_FRACTION) {
            double guardRadius = Math.max(radiusEstimate, minorAxis * (CLUMP_FRAGMENT_RADIUS_FRACTION + 0.01));
            double guardedAreaEstimate = area / Math.max(1.0, Math.PI * guardRadius * guardRadius * CLUMP_AREA_FILL_ESTIMATE);
            double guardedLineEstimate = majorAxis / Math.max(1.0, guardRadius * CLUMP_WIDTH_SPACING_RADIUS);
            double guardedRawCount;
            if (majorAxis / minorAxis > 2.8) {
                guardedRawCount = Math.max(guardedAreaEstimate, guardedLineEstimate);
            } else {
                guardedRawCount = Math.max(guardedAreaEstimate, Math.min(guardedLineEstimate, guardedAreaEstimate * 1.35));
            }
            int guardedCount = Math.max(1, (int) Math.round(guardedRawCount));
            if (guardedCount < estimatedCount) {
                estimatedCount = guardedCount;
                usedFragmentGuard = true;
            }
        }

        List<Center> centers = new ArrayList<>();
        if (usedFragmentGuard) {
            if (majorAxis / minorAxis <= 2.4) {
                centers = synthesizeGridMemberCenters(contour, rect, estimatedCount, top);
            } else {
                centers = synthesizeMemberCenters(contour, estimatedCount, Math.max(radiusEstimate, minorAxis * 0.22), top);
            }
        } else if (!selected.isEmpty()) {
            for (int i = 0; i < Math.min(estimatedCount, selected.size()); i++) {
                Peak peak = selected.get(i);
                centers.add(new Center(peak.x, peak.y));
            }
        }
        if (centers.size() < estimatedCount) {
            centers = synthesizeMemberCenters(contour, estimatedCount, radiusEstimate, top);
        }

        double meanEstimate = 0.0;
        for (double estimate : estimates) {
            meanEstimate += estimate;
        }
        meanEstimate /= estimates.size();
        double agreement = 0.0;
        if (meanEstimate > 0.0) {
            double variance = 0.0;
            for (double estimate : estimates) {
                double delta = estimate - meanEstimate;
                variance += delta * delta;
            }
            variance /= estimates.size();
            agreement = 1.0 - clamp(Math.sqrt(variance) / Math.max(1.0, meanEstimate * 0.55), 0.0, 1.0);
        }
        double peakSupport = clamp(peakCount / (double) Math.max(1, estimatedCount), 0.0, 1.0);
        double sizeSupport = clamp(maxRadius / Math.max(1.0, minorAxis * 0.42), 0.0, 1.0);
        double countConfidence = 0.52 * agreement + 0.34 * peakSupport + 0.14 * sizeSupport;
        if (usedFragmentGuard) {
            countConfidence = Math.min(countConfidence, 0.56);
        }

        roiMask.release();
        distance.release();
        dilated.release();
        localMaxMask.release();
        labels.release();
        stats.release();
        centroids.release();
        return new MemberEstimate(centers, estimatedCount, countConfidence, radiusEstimate);
    }

    private List<Center> synthesizeGridMemberCenters(MatOfPoint contour, Rect rect, int estimatedCount, int top) {
        List<Center> points = new ArrayList<>();
        if (estimatedCount <= 0) {
            return points;
        }

        Moments moments = Imgproc.moments(contour);
        if (estimatedCount == 1) {
            if (Math.abs(moments.m00) > 1e-6) {
                points.add(new Center(moments.m10 / moments.m00, moments.m01 / moments.m00 + top));
            } else {
                points.add(new Center(rect.x + rect.width * 0.5, rect.y + rect.height * 0.5 + top));
            }
            return points;
        }

        double aspect = Math.max(0.35, Math.min(2.8, rect.width / Math.max(1.0, (double) rect.height)));
        int columns = Math.max(1, (int) Math.round(Math.sqrt(estimatedCount * aspect)));
        int rows = Math.max(1, (int) Math.ceil(estimatedCount / (double) columns));
        double xMargin = columns > 1 ? 0.20 : 0.5;
        double yMargin = rows > 1 ? 0.24 : 0.5;

        for (int row = 0; row < rows; row++) {
            double rowFraction = rows == 1 ? 0.5 : yMargin + (1.0 - 2.0 * yMargin) * row / (rows - 1);
            for (int column = 0; column < columns; column++) {
                double columnFraction = columns == 1 ? 0.5 : xMargin + (1.0 - 2.0 * xMargin) * column / (columns - 1);
                points.add(new Center(rect.x + rect.width * columnFraction, rect.y + rect.height * rowFraction + top));
                if (points.size() >= estimatedCount) {
                    return points;
                }
            }
        }
        return points;
    }

    private List<Center> synthesizeMemberCenters(MatOfPoint contour, int estimatedCount, double radius, int top) {
        List<Center> points = new ArrayList<>();
        if (estimatedCount <= 0) {
            return points;
        }

        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        RotatedRect rotated = Imgproc.minAreaRect(contour2f);
        contour2f.release();
        if (estimatedCount == 1) {
            points.add(new Center(rotated.center.x, rotated.center.y + top));
            return points;
        }

        double rectW = rotated.size.width;
        double rectH = rotated.size.height;
        double majorLength;
        double angleDegrees;
        if (rectW >= rectH) {
            majorLength = rectW;
            angleDegrees = rotated.angle;
        } else {
            majorLength = rectH;
            angleDegrees = rotated.angle + 90.0;
        }

        double angle = Math.toRadians(angleDegrees);
        double usableSpan = Math.max(radius * (estimatedCount - 1), majorLength - radius * 1.4);
        double step = usableSpan / Math.max(1, estimatedCount - 1);
        double start = -usableSpan * 0.5;
        double ux = Math.cos(angle);
        double uy = Math.sin(angle);

        for (int index = 0; index < estimatedCount; index++) {
            double offset = start + index * step;
            points.add(new Center(rotated.center.x + ux * offset, rotated.center.y + uy * offset + top));
        }
        return points;
    }

    private Mat contourMask(MatOfPoint contour, Rect rect) {
        Mat roiMask = Mat.zeros(rect.height, rect.width, CvType.CV_8UC1);
        Point[] points = contour.toArray();
        Point[] shifted = new Point[points.length];
        for (int i = 0; i < points.length; i++) {
            shifted[i] = new Point(points[i].x - rect.x, points[i].y - rect.y);
        }
        MatOfPoint shiftedContour = new MatOfPoint(shifted);
        List<MatOfPoint> shiftedContours = new ArrayList<>();
        shiftedContours.add(shiftedContour);
        Imgproc.drawContours(roiMask, shiftedContours, -1, new Scalar(255), -1);
        shiftedContour.release();
        return roiMask;
    }

    private double contourPerimeter(MatOfPoint contour) {
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        double perimeter = Imgproc.arcLength(contour2f, true);
        contour2f.release();
        return perimeter;
    }

    private Detection largestClump(List<Detection> candidates) {
        Detection bestClump = null;
        for (Detection detection : candidates) {
            if (detection.estimatedCount <= 1) {
                continue;
            }
            if (bestClump == null || LARGEST_CLUMP_ORDER.compare(detection, bestClump) < 0) {
                bestClump = detection;
            }
        }
        return bestClump;
    }

    private Double colorConfidence(Scalar meanRgb, Scalar meanHsv) {
        double meanR = meanRgb.val[0];
        double meanG = meanRgb.val[1];
        double meanB = meanRgb.val[2];
        double meanH = meanHsv.val[0];
        double meanS = meanHsv.val[1];
        double meanV = meanHsv.val[2];
        double meanYellowScore = Math.min(meanR, meanG) - meanB;
        double redGreenDelta = Math.abs(meanR - meanG);
        if (meanH < 17.0 && meanR - meanG > 105.0) {
            return null;
        }

        double hueScore = 1.0 - clamp(Math.abs(meanH - 27.0) / 24.0, 0.0, 1.0);
        double saturationScore = clamp((meanS - 45.0) / 150.0, 0.0, 1.0);
        double valueScore = clamp((meanV - 35.0) / 170.0, 0.0, 1.0);
        double rgbScore = clamp((meanYellowScore - 20.0) / 95.0, 0.0, 1.0);
        double balanceScore = 1.0 - clamp((redGreenDelta - 28.0) / 95.0, 0.0, 1.0);

        return 0.34 * hueScore
                + 0.20 * saturationScore
                + 0.16 * valueScore
                + 0.16 * rgbScore
                + 0.14 * balanceScore;
    }

    private static double yellowScore(int r, int g, int b, int saturation) {
        double yellow = Math.min(r, g) - b;
        double balanceBonus = Math.max(0, 80 - Math.abs(r - g)) * 0.12;
        double saturationBonus = saturation * 0.08;
        return yellow + balanceBonus + saturationBonus;
    }

    private void drawLabel(Canvas canvas, String label, float x, float y, int color) {
        textPaint.setColor(color);
        float textWidth = textPaint.measureText(label);
        float clampedX = Math.max(4.0f, Math.min(x, canvas.getWidth() - textWidth - 10.0f));
        float clampedY = Math.max(38.0f, Math.min(y, canvas.getHeight() - 8.0f));
        canvas.drawRect(clampedX - 5.0f, clampedY - 34.0f, clampedX + textWidth + 8.0f, clampedY + 8.0f, textBackgroundPaint);
        canvas.drawText(label, clampedX, clampedY, textPaint);
    }

    private static double meanX(List<Center> centers) {
        double sum = 0.0;
        for (Center center : centers) {
            sum += center.x;
        }
        return sum / Math.max(1, centers.size());
    }

    private static double meanY(List<Center> centers) {
        double sum = 0.0;
        for (Center center : centers) {
            sum += center.y;
        }
        return sum / Math.max(1, centers.size());
    }

    private static double medianRadius(List<Peak> peaks, double fallback) {
        if (peaks.isEmpty()) {
            return fallback;
        }
        int count = Math.min(8, peaks.size());
        List<Double> radii = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            radii.add(peaks.get(i).radius);
        }
        Collections.sort(radii);
        int middle = radii.size() / 2;
        if (radii.size() % 2 == 1) {
            return radii.get(middle);
        }
        return (radii.get(middle - 1) + radii.get(middle)) * 0.5;
    }

    private static int computeRoiTop(int height) {
        return Math.max(0, Math.min(height - 1, (int) Math.round(height * ROI_TOP_FRACTION)));
    }

    private static int oddKernelSize(int value, int minimum) {
        int size = Math.max(minimum, value);
        return size % 2 == 0 ? size + 1 : size;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static String formatShort(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    public static class Detection {
        public final double centerX;
        public final double centerY;
        public final double areaPx;
        public final int boundX;
        public final int boundY;
        public final int boundW;
        public final int boundH;
        public final int estimatedCount;
        public final double steeringError;
        public final double confidence;
        public final double fillRatio;
        public final double aspectRatio;
        public final double radiusPx;
        public final double circularity;
        public final double extent;
        public final double maskCoverage;
        public final double countConfidence;
        public final List<Center> memberCenters;
        public final boolean isClump;

        private Detection(double centerX, double centerY, double areaPx,
                          int boundX, int boundY, int boundW, int boundH,
                          int estimatedCount, double steeringError,
                          double confidence, double fillRatio, double aspectRatio,
                          double radiusPx, double circularity, double extent,
                          double maskCoverage, double countConfidence,
                          List<Center> memberCenters, boolean isClump) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.areaPx = areaPx;
            this.boundX = boundX;
            this.boundY = boundY;
            this.boundW = boundW;
            this.boundH = boundH;
            this.estimatedCount = estimatedCount;
            this.steeringError = steeringError;
            this.confidence = confidence;
            this.fillRatio = fillRatio;
            this.aspectRatio = aspectRatio;
            this.radiusPx = radiusPx;
            this.circularity = circularity;
            this.extent = extent;
            this.maskCoverage = maskCoverage;
            this.countConfidence = countConfidence;
            this.memberCenters = Collections.unmodifiableList(new ArrayList<>(memberCenters));
            this.isClump = isClump;
        }

        public String direction() {
            if (steeringError < -0.03) {
                return "left";
            }
            if (steeringError > 0.03) {
                return "right";
            }
            return "center";
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "Detection{~%d center=(%.0f, %.0f) err=%s area=%.0f conf=%.2f}",
                    estimatedCount, centerX, centerY, formatShort(steeringError), areaPx, confidence);
        }
    }

    public static class Center {
        public final double x;
        public final double y;

        private Center(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private static class Peak {
        final double radius;
        final double x;
        final double y;

        Peak(double radius, double x, double y) {
            this.radius = radius;
            this.x = x;
            this.y = y;
        }
    }

    private static class PollenBall {
        final double centerX;
        final double centerY;
        final double radiusPx;
        final double confidence;

        PollenBall(double centerX, double centerY, double radiusPx, double confidence) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.radiusPx = radiusPx;
            this.confidence = confidence;
        }
    }

    private static class MemberEstimate {
        final List<Center> centers;
        final int estimatedCount;
        final double countConfidence;
        final double radiusPx;

        MemberEstimate(List<Center> centers, int estimatedCount, double countConfidence, double radiusPx) {
            this.centers = centers;
            this.estimatedCount = estimatedCount;
            this.countConfidence = countConfidence;
            this.radiusPx = radiusPx;
        }
    }
}
