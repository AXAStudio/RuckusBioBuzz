package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


public class colorTuner implements VisionProcessor {

    int x, y, x2, y2;

    public int frameWidth, frameHeight;
    public List<Integer> xList = new ArrayList<>();

    int lastTopY;

    int lastLeftX;
    public int r;
    public int g;
    public int b;
    private final Telemetry telemetry;
    public colorTuner(Telemetry telemetry) {
        this.telemetry = telemetry;
    }




    //Not used but neccesary for
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameHeight = height;
        lastTopY = height;
        frameWidth = width;
        lastLeftX = width;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanoes) {
        if (frameWidth == 0 || frameHeight == 0) return null;
        if (frame == null || frame.empty()) return null;

        Mat rgb = new Mat();
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);

        x  = (int)(frameWidth  * 0.4f);
        x2 = (int)(frameWidth  * 0.6f);
        y  = (int)(frameHeight * 0.4f);
        y2 = (int)(frameHeight * 0.6f);

        if (x < 0 || y < 0 || x2 > rgb.cols() || y2 > rgb.rows()) {
            rgb.release();
            return null;
        }

        Mat patch = rgb.submat(y, y2, x, x2);
        Scalar mean = Core.mean(patch);
        r = (int) mean.val[0];
        g = (int) mean.val[1];
        b = (int) mean.val[2];

        telemetry.addData("r", r);
        telemetry.addData("g", g);
        telemetry.addData("b", b);
        telemetry.update();

        patch.release();
        rgb.release();
        return null;
    }
    //also not used
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Paint paint = new Paint();
        paint.setColor(Color.BLACK);
        paint.setStrokeWidth(3f);
        paint.setStyle(Paint.Style.STROKE);

        float left   = onscreenWidth  * 0.4f;
        float right  = onscreenWidth  * 0.6f;
        float top    = onscreenHeight * 0.4f;
        float bottom = onscreenHeight * 0.6f;

        canvas.drawRect(left, top, right, bottom, paint);
    }
}
