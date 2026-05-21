package org.firstinspires.ftc.teamcode.Tools;

import org.jetbrains.annotations.UnknownNullability;

import java.util.List;

public class getMedian {
    public static double getMedian(List<Integer> data) {
        if (data.size() % 2 == 0) {
            return (data.get(data.size() / 2) + data.get((data.size() / 2) - 1)) / 2.0;
        }
        return data.get(data.size() / 2);
    }
    public static double getMedianDown(List<Integer> data) {
        if (data.size() % 2 == 0) {
            return data.get((data.size()/2)-1);
        }
        return data.get(data.size() / 2);
    }
}