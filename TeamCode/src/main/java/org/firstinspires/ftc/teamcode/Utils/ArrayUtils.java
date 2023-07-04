package org.firstinspires.ftc.teamcode.Utils;

public class ArrayUtils {
    public static double[] minIndex(double[] array) {
        double minIndex = 0;
        double minValue = Double.MAX_VALUE;
        for (int i = 0; i < array.length; i++) {
            if (array[i] < minValue) {
                minIndex = i;
                minValue = array[i];
            }
        }
        return new double[]{minIndex, minValue};
    }
}
