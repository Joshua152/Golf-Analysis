package util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.RecursiveTask;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class BezierFitRANSAC extends BezierFit {
    /**
     * Finds a bezier fit using a RANSAC method
     * @param points All the points, including outliers
     * @param degrees Degrees of the bezier polynomial
     * @param s Min number of points to it the model
     * @param t Distance threshold
     * @param n Number of samples
     */
    public BezierFitRANSAC(ArrayList<Point> points, int degrees, int s, int t, int n) {
        super();

        this.points = points;
        this.degrees = degrees;
//
//        ForkJoinPool pool = ForkJoinPool.commonPool();
//        RANSACGroup group = pool.invoke(new RANSACRecursiveAction(s, t, n));

//        copy(group.fit);
        AtomicInteger numDone = new AtomicInteger(0);
        int maxInliers = 0;

        BezierFit best = null;

//        // use 10 threads
//        for(int i = 0; i < 10; i++) {
//            new Thread(() -> {
                for(int j = 0; j < n / 10; j++) {
                    for(int k = 0; k < 7; k++)
                        System.out.print("\b");

                    System.out.printf("%.2f%%", (numDone.get() / (double) n) * 100);

                    ArrayList<Point> subsetPoints = new ArrayList<Point>();
                    TreeSet<Integer> chosen = new TreeSet<Integer>();

                    for(int k = 0; k < s; k++)
                        chosen.add((int) (Math.random() * (points.size() / s)) + (k * points.size() / s));

                    for(Iterator<Integer> iterator = chosen.iterator(); iterator.hasNext(); ) {
                        int num = iterator.next();

                        subsetPoints.add(points.get(num));
                    }

                    BezierFit curr = new BezierFit(subsetPoints, degrees);

                    int numInliers = getNumInliers(t);
                    if(numInliers > maxInliers) {
                        maxInliers = numInliers;
                        best = curr;
                    }

                }
//            copy(best);
//            }).start();
//        }
    }

    /**
     * Gets the number of inliers for RANSAC (number of points within epsilon distance)
     * @param distThreshold The max distance to be considered an inlier
     * @return Returns the number of points which are within the epsilon threshold
     */
    private int getNumInliers(int distThreshold) {
        int numInliers = 0;

        for(Point p : points) {
            double dist = getMinDistance(p, 1e-6);

            if(dist <= distThreshold)
                numInliers++;
        }

        return numInliers;
    }

    public void visualizeMinDist(Mat mat) {
        int[] pascal = getPascalNums(degrees);

        for(Point p : points) {
            Point pCurve = getPoint(pascal, getMinDistance(p, 1e-3));

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
     *
     */
    protected double getMinDistance(Point p, double epsilon) {
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
            if(sdp < sdm)
                minBound = mid;
            else
                maxBound = mid;
        }

        // TODO CHANGE TO Math.sqrt(minDist)
//        return minT; // uncomment if using visualizeMinDist
        return Math.sqrt(minDist);
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

    class RANSACRecursiveAction extends RecursiveTask<RANSACGroup> {
        private int s;
        private int t;
        private int n;

        private RANSACRecursiveAction(int s, int t, int n) {
            this.s = s;
            this.t = t;
            this.n = n;
        }

        @Override
        protected RANSACGroup compute() {
            RANSACGroup best = new RANSACGroup(null, 0);

            if(n > 1000) {
                RANSACRecursiveAction sub1 = new RANSACRecursiveAction(s, t, n / 2);
                RANSACRecursiveAction sub2 = new RANSACRecursiveAction(s, t, n / 2);

                sub1.fork();
                sub2.fork();

                RANSACGroup group1 = sub1.join();
                RANSACGroup group2 = sub2.join();

                if(group1.numInliers > best.numInliers)
                    best = group1;

                if(group2.numInliers > best.numInliers)
                    best = group2;
            } else {
                best = process();
            }
//                return ForkJoinTask.invokeAll(createSubtasks()).stream().min((o1, o2) -> {
//                    if(o1.)
//                });

            return best;
        }

        private RANSACGroup process() {
            RANSACGroup group = new RANSACGroup(null, 0);
//            BezierFit best = null;
//            int maxInliers = 0;

            for(int i = 0; i < n; i++) {
//                for(int k = 0; k < 7; k++)
//                    System.out.print("\b");
//
//                System.out.printf("%.2f%%", (numDone.get() / (double) n) * 100);

                ArrayList<Point> subsetPoints = new ArrayList<Point>();
                TreeSet<Integer> chosen = new TreeSet<Integer>();

                for(int j = 0; j < s; j++)
                    chosen.add((int) (Math.random() * (points.size() / s)) + (j * points.size() / s));

                for(Iterator<Integer> iterator = chosen.iterator(); iterator.hasNext(); ) {
                    int num = iterator.next();

                    subsetPoints.add(points.get(num));
                }

                BezierFit curr = new BezierFit(subsetPoints, degrees);

                int numInliers = getNumInliers(t);
                if(numInliers > group.numInliers) {
                    group.fit = curr;
                    group.numInliers = numInliers;
                }
            }

            return group;
        }
    }

    class RANSACGroup {
        private BezierFit fit;
        private int numInliers;

        private RANSACGroup(BezierFit fit, int numInliers) {
            this.fit = fit;
            this.numInliers = numInliers;
        }
    }
}
