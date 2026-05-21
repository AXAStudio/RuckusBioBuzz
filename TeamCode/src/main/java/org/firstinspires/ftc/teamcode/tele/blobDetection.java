package org.firstinspires.ftc.teamcode.tele;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import android.graphics.Canvas;

public class blobDetection implements VisionProcessor {
    public String hex = "";
    public int x,y,x2,y2;
    public int frameWidth, frameHeight;
    public List<Integer> xyList = new ArrayList<>();
    //Not used but neccesary for
    @Override
    public void init(int width, int height, CameraCalibration calibration){
        frameHeight = height;
        frameWidth = width;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanoes) {
        xyList.clear();
        int res = 15;


        Mat rgb = new Mat();
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_YCrCb2RGB);
        for (int i = 0; i < res; i++) {
            for(int u =0; u < res; u++) {
                x = frameWidth * i / res;
                y = frameHeight * u / res;
                x2 = x + frameWidth / res;
                y2 = y + frameHeight / res;
                Mat patch = rgb.submat(y, y2, x, x2);
                Scalar mean = Core.mean(patch);
                int r = (int) mean.val[0];
                int g = (int) mean.val[1];
                int b = (int) mean.val[2];
                if (r > 150 && g > 150 && b < 50) {
                    xyList.add(x);
                    xyList.add(y);
                    xyList.add(x2);
                    xyList.add(y2);
                }
            }
        }
        return xyList;
    }
    //also not used
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}


