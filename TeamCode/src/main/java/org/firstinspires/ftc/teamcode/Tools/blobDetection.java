package org.firstinspires.ftc.teamcode.Tools;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

public class blobDetection implements VisionProcessor {
    private static final int GRID_RESOLUTION = 15;
    private static final int RED_MIN = 150;
    private static final int GREEN_MIN = 150;
    private static final int BLUE_MAX = 50;

    public int frameWidth, frameHeight;
    public final List<Integer> xyList = new ArrayList<>();
    public final List<Integer> xList = new ArrayList<>();
    public double median = Double.NaN;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameHeight = height;
        frameWidth = width;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanoes) {
        xyList.clear();
        xList.clear();

        for (int column = 0; column < GRID_RESOLUTION; column++) {
            for (int row = 0; row < GRID_RESOLUTION; row++) {
                int x = frameWidth * column / GRID_RESOLUTION;
                int y = frameHeight * row / GRID_RESOLUTION;
                int x2 = frameWidth * (column + 1) / GRID_RESOLUTION;
                int y2 = frameHeight * (row + 1) / GRID_RESOLUTION;

                Mat patch = frame.submat(y, y2, x, x2);
                Scalar mean = Core.mean(patch);
                patch.release();

                int r = (int) mean.val[0];
                int g = (int) mean.val[1];
                int b = (int) mean.val[2];
                if (r > RED_MIN && g > GREEN_MIN && b < BLUE_MAX) {
                    xyList.add(x);
                    xyList.add(y);
                    xyList.add(x2);
                    xyList.add(y2);
                    xList.add(x);
                    xList.add(x2);
                }
            }
        }

        median = xList.isEmpty() ? Double.NaN : getMedian.getMedianDown(xList);
        return xyList;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
