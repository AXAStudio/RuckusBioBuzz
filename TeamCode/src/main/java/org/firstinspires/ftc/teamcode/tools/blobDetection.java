package org.firstinspires.ftc.teamcode.tools;

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

public class blobDetection implements VisionProcessor {
    public String hex = "";
    int x, y, x2, y2;
    public double rawAngle;
    public int frameWidth, frameHeight;
    public List<Integer> xList = new ArrayList<>();
    public double median;
    int res = 15;
    int lastTopY;
    int lastBottomY= 0;
    int lastLeftX;
    int lastRightX = 0;


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
        for (int i = 0; i < res; i++) {
            for (int u = 0; u < res; u++) {
                x = Math.round(frameWidth * i / res);
                y = Math.round(frameHeight * u / res);
                x2 = Math.round(x + frameWidth / res);
                y2 = Math.round(y + frameHeight / res);
                Mat patch = rgb.submat(y, y2, x, x2);
                Scalar mean = Core.mean(patch);
                int r = (int) mean.val[0];
                int g = (int) mean.val[1];
                int b = (int) mean.val[2];

                if (r > 150 && g > 150 && b < 50) {
                    xList.add((x+x2)/2);
                    //UNNECESSARY TESTING MASK EFFICIENCY BOOST:
                    if(y < lastTopY) {
                        lastTopY = y;
                    }
                    if(y2 > lastBottomY){
                        lastBottomY = y2;
                        //End
                    }
                    if(x< lastLeftX ){
                        lastLeftX =x;
                    }
                    if(x2< lastRightX ){
                        lastRightX = x2;
                    }

                }
            }
        }
        //Quartile Logic
        //Get Median
        rgb.release();
        Mat patch = rgb.submat(y, y2, x, x2);
        Scalar mean = Core.mean(patch);
        patch.release();  // add this
        median = getMedian.getMedianDown(xList);
        rawAngle = -(frameWidth/2-median);

        return null;
    }
    //also not used
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        int lineWidth = 5;
        android.graphics.Paint paint = new android.graphics.Paint();
// where you should put the line stuff
// Correct parameter order: (startX, startY, stopX, stopY)
        paint.setColor(Color.BLACK);
        paint.setStrokeWidth(lineWidth);
        paint.setStyle(android.graphics.Paint.Style.FILL);
        canvas.drawLine(lastLeftX - lineWidth, lastTopY - lineWidth, lastLeftX - lineWidth, lastBottomY + lineWidth, paint);
        canvas.drawLine(lastLeftX - lineWidth, lastTopY - lineWidth, lastRightX + lineWidth, lastTopY - lineWidth, paint);
        canvas.drawLine(lastRightX + lineWidth, lastTopY - lineWidth, lastRightX + lineWidth, lastBottomY + lineWidth, paint);
        canvas.drawLine(lastLeftX - lineWidth, lastBottomY + lineWidth, lastRightX + lineWidth, lastBottomY + lineWidth, paint);
        android.graphics.Paint paint2 = new android.graphics.Paint();
        paint.setColor(Color.RED);
        paint.setStrokeWidth(lineWidth);
        paint.setStyle(android.graphics.Paint.Style.FILL);
        canvas.drawLine((float)median,0,(float)median, (float)frameHeight, paint2);
    }
}
