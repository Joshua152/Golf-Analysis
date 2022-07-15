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

            for(int j = 0; j < s; j++)
                chosen.add((int) (Math.random() * (points.size() / s)) + (j * points.size() / s));

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
}