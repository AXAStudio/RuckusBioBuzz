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
        xList.clear();


        Mat rgb = new Mat();
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_YCrCb2RGB);
        x = frameWidth/2-frameWidth/10;
        x2 = frameWidth/2+frameWidth/10;
        y = frameHeight/2-frameHeight/10;
        y2 = frameHeight/2+frameHeight/10;
        Mat patch = rgb.submat(y, y2, x, x2);
        Scalar mean = Core.mean(patch);
        r= (int)mean.val[0];
        g= (int)mean.val[1];
        b= (int)mean.val[2];
        telemetry.addData("r", r);
        telemetry.addData("g", g);
        telemetry.addData("b", b);


        //Get Median
        rgb.release();
        patch.release();  // add this

        return null;
    }
    //also not used
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
