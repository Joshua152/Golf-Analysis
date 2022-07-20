package util;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.TreeSet;
import java.util.concurrent.RecursiveTask;

public class BezierRANSAC extends RecursiveTask<BezierFit> {
    private ArrayList<Point> points;
    private int degrees;

    private int s;
    private int t;
    private int n;

    private int originalN;

    private int maxInliers;

    private double[] ti;

    public BezierRANSAC(ArrayList<Point> points, int degrees, int s, int t, int n) {
//        this.points = points;
//        this.degrees = degrees;
//
//        this.s = s;
//        this.t = t;
//        this.n = n;
//
//        maxInliers = 0;

        this(points, degrees, s, t, n, n);
    }

    public BezierRANSAC(ArrayList<Point> points, int degrees, int s, int t, int n, int originalN) {
        this.points = points;
        this.degrees = degrees;

        this.s = s;
        this.t = t;
        this.n = n;

        this.originalN = originalN;

        maxInliers = 0;

        ti = new double[points.size()];

        setUp();
    }

    private void setUp() {
        double[] dPartialSums = new double[points.size() + 1];

        for(int i = 2; i < dPartialSums.length; i++) {
            double dx = points.get(i - 1).x - points.get(i - 2).x;
            double dy = points.get(i - 1).y - points.get(i - 2).y;

            dPartialSums[i] = dPartialSums[i - 1] + Math.sqrt(dx * dx + dy * dy);
        }

        for(int i = 0; i < ti.length; i++)
            ti[i] = dPartialSums[i + 1] / dPartialSums[dPartialSums.length - 1];
    }

    @Override
    protected BezierFit compute() {
        if(n <= 1000)
            return directCompute();

        BezierRANSAC sub1 = new BezierRANSAC(points, degrees, s, t, n / 2, originalN);
        BezierRANSAC sub2 = new BezierRANSAC(points, degrees, s, t, n / 2, originalN);

        sub1.fork();
        sub2.fork();

        BezierFit fit1 = sub1.join();
        BezierFit fit2 = sub2.join();

        if(sub1.maxInliers > sub2.maxInliers)
            return fit1;

        return fit2;
    }

    private BezierFit directCompute() {
        BezierFit best = null;

        int numDone = 0;

        for(int i = 0; i < n; i++) {
//            for(int j = 0; j < 7; j++)
//                System.out.print("\b");
//
//            System.out.printf("%.2f%%", (numDone / (double) n) * 100);

            ArrayList<Point> subsetPoints = new ArrayList<Point>();
            TreeSet<Integer> chosen = new TreeSet<Integer>();

            // look for start and end of wanted time zone and pick random from that subset
            int endIndex = 0;
            for(int j = 0; j < s; j++) {
                chosen.add((int) (Math.random() * (points.size() / s)) + (j * points.size() / s));

//                int startIndex = endIndex;
//
//                endIndex = getTimeIndex((j + 1.0) / s, startIndex, ti.length - 1);
//
////                System.out.println(startIndex + " " + endIndex);
//
//                chosen.add((int) (Math.random() * (endIndex - startIndex + 1) + startIndex));
            }

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

    /**
     * Gets the index of the nearest time in the ti list
     * @param time The normalized time of the point [0, 1]
     * @param start Index to start search
     * @param end Index for end bound of search
     * @return Returns the closest index correlating to the given time
     */
    private int getTimeIndex(double time, int start, int end) {
       while(start < end) {
           int mid = start + (end - start) / 2;

           if(ti[mid] <= time)
               start = mid + 1;
           else
               end = mid;
       }

       return start;
    }
}