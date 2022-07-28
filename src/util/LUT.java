package util;

import org.opencv.core.Point;

import java.util.Arrays;

public class LUT {
    private Point[] points;

    /**
     * Constructor for a look up table that uses linear interpolation
     * @param points Known points to interpolate between; should contain extremes
     */
    public LUT(Point[] points) {
        this.points = points;

        System.out.println("LUT: " + Arrays.toString(points));
    }

    /**
     * Uses the look up table to approximate y(x)
     * @param x The input
     * @return The approximate value of y
     */
    public double get(double x) {
        System.out.println("x: " + x);

        if(x < points[0].x || x > points[points.length - 1].x)
            System.out.println("value x (" + x + ") out of bounds for [" + points[0].x + ", " + points[points.length - 1].x + "]");

        int start = 0;
        int end = points.length - 1;

        while(start < end) {
            int mid = start + (end - start) / 2;
            Point p = points[mid];

            if(p.x == x)
                return points[mid].y;

            if(p.x <= x)
                start = mid + 1;
            else
                end = mid;
        }

        // LERP
        Point upper = points[start];
        Point lower = points[start - 1];

        double percent = (x - lower.x) / (upper.x - lower.x);

        return lower.y + percent * (upper.y - lower.y);
    }
}
