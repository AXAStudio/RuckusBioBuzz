package org.firstinspires.ftc.teamcode.tools;

import static java.lang.Double.NaN;

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

@Disabled

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
        lastLeftX = frameWidth;
        lastTopY = frameHeight;
        lastBottomY= 0;
        lastRightX = 0;


        Mat rgb = new Mat();
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);
        for (int i = 0; i < res; i++) {
            for (int u = 0; u < res; u++) {
                x = (int)Math.round(frameWidth * i / res);
                y = (int)Math.round(frameHeight * u / res);
                x2 = (int)Math.round(x + frameWidth / res);
                y2 = (int)Math.round(y + frameHeight / res);
                Mat patch = rgb.submat(y, y2, x, x2);
                Scalar mean = Core.mean(patch);
                int r = (int) mean.val[0];
                int g = (int) mean.val[1];
                int b = (int) mean.val[2];

                if (r > 130 && g > 130 && b < 100) { // must tune on field before using this code
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
                    if(x2> lastRightX ){
                        lastRightX = x2;
                    }

                }
            }
        }
        //Quartile Logic
        //Get Median
        rgb.release();
        if(!xList.isEmpty()) {
            median = getMedian.getMedianDown(xList);
            rawAngle = -(frameWidth/2-median);
        }else{
            rawAngle = NaN;
        }



        return null;
    }
    //also not used
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        int lineWidth = 5;
        android.graphics.Paint paint = new android.graphics.Paint();
        paint.setColor(Color.BLACK);
        paint.setStrokeWidth(lineWidth);
        paint.setStyle(Paint.Style.STROKE);
        if(!Double.isNaN(rawAngle) && !xList.isEmpty()) {
            canvas.drawRect((lastLeftX - lineWidth) * scaleBmpPxToCanvasPx, (lastTopY - lineWidth) * scaleBmpPxToCanvasPx, (lastRightX + lineWidth) * scaleBmpPxToCanvasPx, (lastBottomY + lineWidth) * scaleBmpPxToCanvasPx, paint);
        }
        int lineWidth2 = 10;
        Paint paint2 = new Paint();
        paint2.setColor(Color.RED);
        paint2.setStrokeWidth(lineWidth2);
        paint2.setStyle(Paint.Style.FILL);
        if(!Double.isNaN(rawAngle) && !xList.isEmpty()) {
            canvas.drawLine((float) median*scaleBmpPxToCanvasPx, 0, (float) median*scaleBmpPxToCanvasPx, (float) frameHeight*scaleBmpPxToCanvasPx, paint2);
        }

    }
}

