package util;

import data.TimedPoint;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.io.Serializable;
import java.lang.reflect.Array;
import java.math.BigDecimal;
import java.math.MathContext;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleToLongFunction;

// Following https://www.jimherold.com/computer-science/best-fit-bezier-curve
public class BezierFit implements Serializable {
    public ArrayList<? extends Point> points;
    protected Mat x;
    protected Mat y;

    protected int degrees;

    private double[] ti;

    protected Mat px;
    protected Mat py;

    public BezierFit(ArrayList<? extends Point> points, int degrees) {
        this.points = points;

        x = new Mat(points.size(), 1, CvType.CV_32F);
        y = new Mat(points.size(), 1, CvType.CV_32F);

        this.degrees = degrees;

        ti = new double[points.size()];

        px = null;
        py = null;

        double[] xBuffer = new double[points.size()];
        double[] yBuffer = new double[points.size()];

        for(int i = 0; i < points.size(); i++) {
            xBuffer[i] = points.get(i).x;
            yBuffer[i] = points.get(i).y;
        }

        x.put(0, 0, xBuffer);
        y.put(0, 0, yBuffer);

        double[] dPartialSums = new double[points.size() + 1];

        for(int i = 2; i < dPartialSums.length; i++) {
            double dx = x.get(i - 1, 0)[0] - x.get(i - 2, 0)[0];
            double dy = y.get(i - 1, 0)[0] - y.get(i - 2, 0)[0];

            dPartialSums[i] = dPartialSums[i - 1] + Math.sqrt(dx * dx + dy * dy);
        }

        for(int i = 0; i < ti.length; i++)
            ti[i] = dPartialSums[i + 1] / dPartialSums[dPartialSums.length - 1];

        Mat tn = new Mat(points.size(), degrees + 1, CvType.CV_32F);
        double[] tnBuffer = new double[(int) (tn.rows() * tn.cols())];

        for(int i = 0; i < tn.rows(); i++) {
            for(int j = 0; j < tn.cols(); j++)
                tnBuffer[i * tn.cols() + j] = Math.pow(ti[i], degrees - j);
        }

        tn.put(0, 0, tnBuffer);

        // solve for Py and Px
        Mat m = new Mat(degrees + 1, degrees + 1, CvType.CV_32F);

        int[] globalPascal = getPascalNums(degrees);
        for(int i = 0; i < m.rows(); i++) {
            int[] pascal = getPascalNums(degrees - i);

            float[] mRow = new float[degrees + 1];
            for(int j = 0; j < pascal.length; j++) {
                    int sign = (int) Math.pow(-1, i + j + degrees);

                    mRow[j] = sign * globalPascal[i] * pascal[j];
            }

            m.put(i, 0, mRow);
        }

        Mat minv;

        if(Core.determinant(m) == 0)
            minv = m.inv(Core.DECOMP_SVD);
        else
            minv = m.inv();

        Mat a = tn.t().matMul(tn);

        if(Core.determinant(a) == 0)
            a = a.inv(Core.DECOMP_SVD);
        else
            a = a.inv();

        Mat coeff = minv.matMul(a.matMul(tn.t()));

        px = coeff.matMul(x);
        py = coeff.matMul(y);
    }

    public static BezierFit RANSAC(ArrayList<Point> points, int degrees, int s, int t, int n) {
        int numDone = 0;
        int maxInliers = 0;
        BezierFit best = null;

        for(int j = 0; j < n; j++) {
            for(int k = 0; k < 7; k++)
                System.out.print("\b");

            System.out.printf("%.2f%%", (numDone / (double) n) * 100);

            ArrayList<Point> subsetPoints = new ArrayList<Point>();
            TreeSet<Integer> chosen = new TreeSet<Integer>();

            for(int k = 0; k < s; k++)
                chosen.add((int) (Math.random() * (points.size() / s)) + (k * points.size() / s));

            for(Iterator<Integer> iterator = chosen.iterator(); iterator.hasNext(); ) {
                int num = iterator.next();

                subsetPoints.add(points.get(num));
            }

            BezierFit curr = new BezierFit(subsetPoints, degrees);

            int numInliers = curr.getNumInliers(t);
            if(numInliers > maxInliers) {
                maxInliers = numInliers;
                best = curr;
            }

            numDone++;
        }

        return best;
    }

    public static BezierFit RANSACRecursive(ArrayList<TimedPoint> points, int degrees, int s, int t, int n) {
        ForkJoinPool pool = ForkJoinPool.commonPool();

        return pool.invoke(new BezierRANSAC(points, degrees, s, t, n));
    }

    /**
     * Gets points on the Bezier curve based on the given increment dt
     * @param dt Increment for points (0, 1]
     * @return Returns points that lie along the Bezier curve
     */
    public Point[] getPoints(double dt) {
        Point[] points = new Point[(int) (1 / dt)];

        int[] pascal = getGlobalPascal();

//        for(double t = 0; t < 1; t += dt) {
//            points[(int) ((t / dt) + 0.000001)] = getPoint(pascal, t);
//        }

        BigDecimal t = new BigDecimal(-dt);
        for(int i = 0; i < 1 / dt; i++) {
            t = t.add(BigDecimal.valueOf(dt));

//            System.out.println("t: " + t + "| " + t.divide(BigDecimal.valueOf(dt), MathContext.DECIMAL32).intValue() + " " + getPoint(pascal, t.doubleValue()));

            points[t.divide(BigDecimal.valueOf(dt), MathContext.DECIMAL32).intValue()] = getPoint(pascal, t.doubleValue());
        }

        return points;
    }

    /**
     * Gets the point given the value t (generates pascal numbers within method)
     * @param t The t value [0, 1]
     * @return Returns the point that is represented by t
     */
    public Point getPoint(double t) {
        return getPoint(getGlobalPascal(), t);
    }

    /**
     * Used to get a single point on a bezier curve
     * @param pascal The Pascal coefficients (row on pascal's triangle is num control points - 1)
     * @param t Normalized time index of point
     */
    public Point getPoint(int[] pascal, double t) {
        double xVal = 0;
        double yVal = 0;

        for(int i = 0; i < pascal.length; i++) {
            double shared = pascal[i] * Math.pow(1 - t, pascal.length - i - 1) * Math.pow(t, i);

            xVal += px.get(i, 0)[0] * shared;
            yVal += py.get(i, 0)[0] * shared;
        }

//        System.out.printf("(%f, %f), ", xVal, yVal);

        return new Point(xVal, yVal);
    }

    /**
     * Creates a lookup table to allow the curve as a function interms of x
     * @param dt Spacing for the concrete values in the LUT
     * @return Returns a LUT that represents the Bezier curve
     */
    public LUT createLUT(double dt) {
        Point[] points = new Point[(int) ((1 / dt) + 0.5)];

        int[] pascal = getGlobalPascal();

        double t = 0;
        for(int i = 0; i < points.length; i++) {
            points[i] = getPoint(pascal, t);

            t += dt;
        }

        return new LUT(points);
    }

    /**
     * Gets the number of inliers for RANSAC (number of points within epsilon distance)
     * @param distThreshold The max distance to be considered an inlier
     * @return Returns the number of points which are within the epsilon threshold
     */
    protected int getNumInliers(int distThreshold) {
        int numInliers = 0;

        for(Point p : points) {
            double dist = getMinDistance(p, 1e-6)[0];

            if(dist <= distThreshold)
                numInliers++;
        }

        return numInliers;
    }

    public void visualizeMinDist(Mat mat) {
        int[] pascal = getPascalNums(degrees);

        for(Point p : points) {
            Point pCurve = getPoint(pascal, getMinDistance(p, 1e-3)[0]);

            Scalar color = null;
            if(Math.sqrt(squareDist(p, pCurve)) <= 10)
                color = new Scalar(0, 255, 0);
            else
                color = new Scalar(0, 0, 255);

            Imgproc.line(mat, p, pCurve, new Scalar(0, 255, 0));
        }
    }

    /**
     * Gets the minimum distance from a point to the curve
     * @param p Point off the curve
     * @param epsilon Threshold for how precise t should be
     * @return Returns a double[] for the min distance and minT ([minDist, minT])
     */
    public double[] getMinDistance(Point p, double epsilon) {
        double minT = 0;
        double minDist = Double.POSITIVE_INFINITY;

        int[] pascal = getPascalNums(degrees);
        int numChecks = 25;
        for(int i = 0; i < numChecks; i++) {
//            System.out.print(i / (double) numChecks);
            Point pCurve = getPoint(pascal, i / (double) numChecks);

            double sDist = squareDist(p, pCurve);
            if(sDist < minDist) {
                minT = i / (double) numChecks;
                minDist = sDist;
            }
        }

        double minBound = minT - (1 / (double) numChecks);
        double maxBound = minT + (1 / (double) numChecks);
        while(maxBound - minBound > epsilon) {
            double mid = (maxBound + minBound) / 2;
            minT = mid;

            Point midPlus = getPoint(pascal, Math.min(mid + epsilon, 1));
            Point midMinus = getPoint(pascal, Math.max(mid - epsilon, 0));

            double sdp = squareDist(p, midPlus);
            double sdm = squareDist(p, midMinus);
//            System.out.println((sdp < sdm) + " " + sdp + " " + sdm);
            if (sdp < sdm) {
                minBound = mid;
                minDist = sdp;
            } else {
                maxBound = mid;
                minDist = sdm;
            }
        }

        // TODO CHANGE TO Math.sqrt(minDist)
        return new double[] {Math.sqrt(minDist), minT};
    }

    /**
     * Gets pascal for the degrees of the Bezier curve
     * @return Returns an int[] for the pascal numbers
     */
    public int[] getGlobalPascal() {
        return getPascalNums(px.rows() - 1);
    }

    /**
     * Gets numbers from a given row of Pascal's triangle (using combinatorics)
     * @param row Rows are 0 indexed
     * @return Returns int[] of nums from the given row of Pascal's triangle
     */
    protected int[] getPascalNums(int row) {
        int[] nums = new int[row + 1];

        int prev = 1;

        nums[0] = prev;

        for(int i = 1; i <= row; i++) {
            int curr = (prev * (row - i + 1)) / i;

            nums[i] = curr;

            prev = curr;
        }

        return nums;
    }

    /**
     * Finds the squared distance between 2 points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return Returns a double for the squared distance between the points
     */
    private double squareDist(Point p1, Point p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    }

    /**
     * Gets an array to represent the points in terms of time (normalized from [0, 1])
     * @return Retuns an array which represents all the points given in the points ArrayList
     */
    public double[] getTi() {
        return ti;
    }
}
