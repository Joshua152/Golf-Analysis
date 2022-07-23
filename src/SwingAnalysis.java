import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.xfeatures2d.SURF;
import util.BezierFit;

import java.util.*;

public class SwingAnalysis {

    // TODo: SPACING WAS THE CULPRIT
    private Mat trackingLines;
    private double[] clubLine;
    private Point referencePoint;

    private ArrayList<Point> points;
    private ArrayList<Point> backswingPoints;
    private ArrayList<Point> downswingPoints;

    private Point downswingPoint;
    private int downswingFrame;
    private int downswingFrameY;

    private ArrayList<Integer> keyPointBasedTime; // just replace with int for keyPointBasedPoints frame # offset?
    private ArrayList<Point> keyPointBasedPoints;
    private ArrayList<Double> blurredY;

    private int frame;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public SwingAnalysis() {
        trackingLines = new Mat();
        clubLine = new double[4];
        referencePoint = null;

        points = new ArrayList<Point>();
        backswingPoints = new ArrayList<Point>();
        downswingPoints = new ArrayList<Point>();

        downswingFrame = 0;
        downswingFrameY = 0;
        keyPointBasedTime = new ArrayList<Integer>();
        keyPointBasedPoints = new ArrayList<Point>();
        blurredY = new ArrayList<Double>();

        frame = 0;
    }

    public static void main(String[] args) {
        SwingAnalysis swingAnalysis = new SwingAnalysis();
        swingAnalysis.run();
    }

    private void run() {
        VideoCapture capture = new VideoCapture("src/res/tigerdriver.mp4");
        double fps = 120;//capture.get(Videoio.CAP_PROP_FPS);

        Mat prev = new Mat();
        Mat curr = new Mat();

        Mat[] processedBuffer = new Mat[2];
        Mat tracer = new Mat();
        Mat yGraph = new Mat();

        Mat swingtracer = Imgcodecs.imread("tracer.jpg");

        Mat segmentation = new Mat();

        while(true) {
            boolean ok = capture.read(curr);

            frame++;

            if(frame == 515) {
                postprocess();

                BezierFit fit = new BezierFit(points,  10);
                Point[] points = fit.getPoints(0.1);

                tracer = trackingLines.clone();
                for(int i = 1; i < points.length; i++) {
                    Imgproc.line(tracer, points[i - 1], points[i], new Scalar(0, 255, 0));
                    System.out.println("line: " + points[i - 1] + " -> " + points[i]);
                }

                for(Point p : downswingPoints) {
                    System.out.println("new Point(" + p.x + ", " + p.y + "), ");
                }

                yGraph = new Mat(curr.rows(), blurredY.size(), CvType.CV_8UC3);
                for(int i = 0; i < blurredY.size(); i++)
                    Imgproc.circle(yGraph, new Point(i, blurredY.get(i)), 3, new Scalar(255, 255, 70), -1);

                Imgproc.circle(yGraph, new Point(downswingFrame - keyPointBasedTime.get(0), downswingFrameY), 5, new Scalar(255, 255, 255), -1);
                Imgproc.line(tracer, new Point(0, downswingFrameY), new Point(curr.cols() - 1, downswingFrameY), new Scalar(0, 255, 0));
                Imgproc.putText(yGraph, downswingFrame + "", new Point(0, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                Imgproc.putText(yGraph, blurredY.size() + "", new Point(0, 200), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                HighGui.imshow("YGraph", yGraph);

                Imgproc.circle(tracer, downswingPoint, 3, new Scalar(255, 70, 255), -1);

                segmentation = Mat.zeros(curr.size(), CvType.CV_8UC3);
                for(Point p : backswingPoints)
                    Imgproc.circle(segmentation, p, 3, new Scalar(255, 0, 0), -1);

                for(Point p : downswingPoints)
                    Imgproc.circle(segmentation, p, 3, new Scalar(100, 255, 255), -1);
            }

            System.out.println("frame: " + frame);

            if(frame >= 515) {
                HighGui.imshow("Tracer", tracer);
                HighGui.imshow("YGraph", yGraph);
                HighGui.imshow("Segmentation", segmentation);
            }

            // STOP AT FRAME 515

//            if(!ok)
//                writer.close();

            if(trackingLines.empty())
                trackingLines = new Mat(curr.size(), curr.type());

            if(!prev.empty() && !curr.empty()) {
               boolean full = handleBuffer(processedBuffer, curr, prev);

               if(full) {
                   Mat res = track(processedBuffer[1], processedBuffer[0], curr, prev);

                   HighGui.imshow("Preprocess", preprocess(curr, prev));
                   Core.add(res, swingtracer, res);
                   HighGui.imshow("Track", res);
               }

               HighGui.waitKey((int) ((1 / fps) * 1000));
            }

            prev = curr.clone();

            System.out.println("REFERENCE POINT: " + referencePoint);
        }
    }

    private Mat preprocess(Mat curr, Mat prev) {
        Mat res = new Mat();

//        Imgproc.morphologyEx(res, res, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
//                new Size(3, 3)));

//        Mat currCanny = new Mat();
//        Mat prevCanny = new Mat();
//        Imgproc.Canny(curr, currCanny, 100, 200);
//        Imgproc.Canny(prev, prevCanny, 100, 200);

        Core.absdiff(curr, prev, res);

        // sharpen
        Imgproc.cvtColor(res, res, Imgproc.COLOR_BGR2GRAY);

//        Mat dest = new Mat(res.size(), res.type());
//        Core.addWeighted(res, 1.5, dest, -0.5, 0, res);

//        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5, 5), new Point(2, 2));
//        Imgproc.morphologyEx(res, res, Imgproc.MORPH_CLOSE, kernel);

        Imgproc.Canny(res.clone(), res, 80, 200);


        // TODO: WHAT?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?
        // probabilistic Hough line transform
        Mat lines = new Mat();
        Imgproc.HoughLinesP(res, lines, 1, Math.PI / 180, 80, 50, 30);

//        System.out.println(lines.dump());

        Mat houghLinesDest = new Mat();
        Imgproc.cvtColor(res, houghLinesDest, Imgproc.COLOR_GRAY2BGR);

//        Mat houghLinesFiltered = new Mat();
//
//        HashSet<Double> okSlopes = new HashSet<Double>();
//
//        for(int i = 0; i < lines.rows(); i++) {
//            double[] l1 = lines.get(i, 0);
//            double slope1 = (l1[3] - l1[1]) / (l1[2] - l1[0]);
//
//            if(okSlopes.contains(slope1))
//                i++;
//
//            for(int j = 0; j < lines.rows(); j++) {
//                double[] l2 = lines.get(j, 0);
//                double slope2 = (l2[3] - l2[1]) / (l2[2] - l2[0]);
//
//                if(Math.min(slope1, slope2) / Math.max(slope1, slope2) <= 0.95) {
//                    houghLinesFiltered.put(houghLinesFiltered.rows(), 0 , l1);
//
//                    okSlopes.add(slope1);
//
//                    j = lines.rows();
//                }
//            }
//        }

        // use median distance insteads of random?

        double[] l = null;
        double largest = 0;

        System.out.println(lines.rows());

        for(int i = 0; i < lines.rows(); i++) {
            if(lines.get(i, 0).length == 0) {
                i++;

                break;
            }

            double[] currLine = lines.get(i, 0);

            double dist = length(currLine);

            if(dist > largest) {
                largest = dist;

                l = currLine;
            }

            i = lines.rows();
        }

        if(l != null) {
            Imgproc.line(houghLinesDest, new Point(l[0], l[1]), new Point(l[2], l[3]),
                    new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
        }

        clubLine = l;

        if(referencePoint == null && clubLine != null) {
            if(clubLine[1] < clubLine[3])
                referencePoint = new Point(clubLine[0], clubLine[1]);
            else
                referencePoint = new Point(clubLine[2], clubLine[3]);
        }

        HighGui.imshow("Hough Lines", houghLinesDest);

        Imgproc.cvtColor(res, res, Imgproc.COLOR_GRAY2BGR);

        return res;
    }

    private Mat track(Mat curr, Mat prev, Mat currOriginal, Mat prevOriginal) {
        // detect/describe keypoints
//        ORB kpFinder = ORB.create();
        SURF kpFinder = SURF.create(1500, 8, 5, true, false);
//        BriefDescriptorExtractor kpFinder = BriefDescriptorExtractor.create();
//        StarDetector kpDetector =  StarDetector.create();

        MatOfKeyPoint currKP = new MatOfKeyPoint();
        Mat currDescriptors = new Mat();
//        kpFinder.detectAndCompute(curr, new Mat(), currKP, currDescriptors);
        kpFinder.detect(curr, currKP);
        kpFinder.compute(curr, currKP, currDescriptors);

        MatOfKeyPoint prevKP = new MatOfKeyPoint();
        Mat prevDescriptors = new Mat();
//        kpFinder.detectAndCompute(prev, new Mat(), prevKP, prevDescriptors);
        kpFinder.detect(prev, prevKP);
        kpFinder.compute(prev, prevKP, prevDescriptors);

        Mat key = new Mat();
        Features2d.drawKeypoints(curr, currKP, key, Scalar.all(-1), Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS);
//        HighGui.imshow("KeyPoints", key);

        // match kepoints

        if(currDescriptors.empty() || prevDescriptors.empty()) {
            Mat out = new Mat();

            Core.add(curr, trackingLines, out);

            return out;
        }

//        FlannBasedMatcher matcher = FlannBasedMatcher.create();
        BFMatcher matcher = BFMatcher.create(Core.NORM_L2, true);

        MatOfDMatch matches = new MatOfDMatch();
        matcher.match(currDescriptors, prevDescriptors, matches, new Mat()); // query, train

//        System.out.println("club line: " + Arrays.toString(clubLine));

        KeyPoint[] currKPArray = currKP.toArray();
        KeyPoint[] prevKPArray = prevKP.toArray();

        // filter matches

        // Only use points within 1 standard deviatino of the median
        ArrayList<Point> filteredPoints = new ArrayList<Point>();

        if(clubLine != null) {
            for (DMatch m : matches.toArray()) {
                Point p1 = currKPArray[m.queryIdx].pt;
                Point p2 = prevKPArray[m.trainIdx].pt;

                if (Math.abs(p1.x - p2.x) < 20 && Math.abs(p1.y - p2.y) < 20 &&
                        !(p1.x > Math.min(clubLine[0], clubLine[2]) && p1.x < Math.max(clubLine[0], clubLine[2]) &&
                                p1.y > Math.min(clubLine[1], clubLine[3]) && p1.y < Math.max(clubLine[1], clubLine[3]))) {

                    double refDist1 = distance(new Point(clubLine[0], clubLine[1]), referencePoint);
                    double refDist2 = distance(new Point(clubLine[2], clubLine[3]), referencePoint);

                    Point clubHead;
                    Point clubGrip;

                    if (refDist1 > refDist2) {
                        clubHead = new Point(clubLine[0], clubLine[1]);
                        clubGrip = new Point(clubLine[2], clubLine[3]);
                    } else {
                        clubHead = new Point(clubLine[2], clubLine[3]);
                        clubGrip = new Point(clubLine[0], clubLine[1]);
                    }

                    double headDist = distance(p1, clubHead);
                    double gripDist = distance(p1, clubGrip);

                    if (headDist < gripDist) {
//                        Imgproc.line(trackingLines, p1, p2, new Scalar(255, 0, 0), 1);

//                        trackPoints.add(p1);
                        filteredPoints.add(p1);
//                        writer.write("[" + p1.x + ", " + p1.y + "],\n");
                    }
//                    else
//                        Imgproc.line(trackingLines, p1, p2, new Scalar(0, 0, 100), 1);
                }
            }

            if(filteredPoints.size() != 0) {
                Point[] withinMedian = withinMedian(filteredPoints, 5);
                points.addAll(Arrays.asList(withinMedian));
                System.out.println("within median: " + withinMedian.length);

                for(Point p : withinMedian) {
                    Imgproc.circle(trackingLines, p, 3, new Scalar(255, 0, 0), -1);
                }

                Point mid = withinMedian[withinMedian.length / 2];
                keyPointBasedTime.add(frame);
                keyPointBasedPoints.add(mid);
            }
        } else if(keyPointBasedTime.size() != 0) {
            keyPointBasedTime.add(frame);
            keyPointBasedPoints.add(keyPointBasedPoints.get(keyPointBasedPoints.size() - 1));
        }

        Mat out = new Mat();

        Core.add(curr, trackingLines, out);

        if(clubLine != null)
            Imgproc.rectangle(out, new Point(clubLine[0], clubLine[1]), new Point(clubLine[2], clubLine[3]), new Scalar(0, 100, 0));

        if(referencePoint != null)
            Imgproc.circle(out, referencePoint, 5, new Scalar(255, 0, 255));

        return out;
    }

    private void postprocess() {
        // plot y values in array in accordance to time and blur
        for(Point p : keyPointBasedPoints)
            blurredY.add(p.y);

        for(int i = 0; i < 15; i++) {
            Mat blurredYMat = new Mat(blurredY.size(), 1, CvType.CV_32F);

            double[] yBuffer = new double[blurredY.size()];
            for(int j = 0; j < yBuffer.length; j++)
                yBuffer[j] = blurredY.get(j);

            blurredYMat.put(0, 0, yBuffer);

            Imgproc.GaussianBlur(blurredYMat, blurredYMat, new Size(1, 31), 0, 0);

            for(int j = 0; j < blurredYMat.rows(); j++)
                blurredY.set(j, blurredYMat.get(j, 0)[0]);
        }

        for(int i = 3; i < blurredY.size() - 3; i++) {
            double curr = blurredY.get(i);

            if(blurredY.get(i - 2) < curr && blurredY.get(i - 1) < curr &&
                    blurredY.get(i + 1) < curr && blurredY.get(i + 2) < curr) {
                downswingFrame = keyPointBasedTime.get(i);
                downswingFrameY = (int) curr;

                i = blurredY.size();
            }
        }

        int downswingIndex = getInsertIndex(keyPointBasedTime, downswingFrame, 0, keyPointBasedTime.size() - 1);
        if(downswingIndex != 0 && downswingIndex != keyPointBasedTime.size() &&
                keyPointBasedTime.get(downswingIndex) - downswingFrame >= downswingFrame - keyPointBasedTime.get(downswingIndex - 1)) {
            downswingIndex--;
        }

        downswingPoint = keyPointBasedPoints.get(downswingIndex);

        System.out.println("downswingIndex: " + downswingIndex);
        System.out.println("keyPointBasedPoints size: " + keyPointBasedPoints.size());

        int trackPointsIndex = points.size() - 1;
        for(int i = keyPointBasedPoints.size() - 1; i >= downswingIndex; i--) {
            while(!keyPointBasedPoints.get(i).equals(points.get(trackPointsIndex))) {
                downswingPoints.add(0, points.get(trackPointsIndex));
                System.out.println("add");

                trackPointsIndex--;
            }

            System.out.println("done: " + i);

            downswingPoints.add(0, points.get(trackPointsIndex));
            trackPointsIndex--;

            Point curr = keyPointBasedPoints.get(i);
            while(keyPointBasedPoints.get(i).equals(curr))
                i--;

            i++;
        }

        downswingPoints.add(0, points.get(trackPointsIndex));

        for(int i = 0; i < trackPointsIndex; i++)
            backswingPoints.add(points.get(i));

        // remove first 10% of downswing points b/c lots of noise
        int n = (int) (downswingPoints.size() * 0.1);
        for(int i = 0; i < n; i++)
            downswingPoints.remove(0);

        int top = 0;
        int sideTop = 0;
        int sideBottom = 0;
        int bottom = 0;

        for(int i = 0; i < downswingPoints.size(); i++) {
            Point p = downswingPoints.get(i);
            double angle = Math.atan2(referencePoint.y - p.y, p.x - referencePoint.x);

            if(inSection(angle, "top"))
                top++;
            else if(inSection(angle, "side-top"))
                sideTop++;
            else if(inSection(angle, "side-bottom"))
                sideBottom++;
            else if(inSection(angle, "bottom"))
                bottom++;
        }

        int sideTopMultiplier = (int) ((top * 0.5) / sideTop); // 0.4
        int sideBottomMultiplier = (int) ((top * 0.20) / sideBottom); // 25
        int bottomMultiplier = (int) ((top * 0.35) / bottom); // 0.3

        System.out.println("MULTIPLIER (SIDE TOP): " + sideTopMultiplier);
        System.out.println("MULTIPLIER (SIDE BOTTOM): " + sideBottomMultiplier);
        System.out.println("MULTIPLIER (BOTTOM): " + bottomMultiplier);
        System.out.println("TOP: " + top);
        System.out.println("SIDE TOP: " + sideTop);
        System.out.println("SIDE BOTTOM: " + sideBottom);
        System.out.println("BOTTOM: " + bottom);

        for(int i = 0; i < downswingPoints.size(); i++) {
            Point p = downswingPoints.get(i);
            double angle = Math.atan2(referencePoint.y - p.y, p.x - referencePoint.x);

            int currMultiplier = 1;

            if(inSection(angle, "side-top"))
                currMultiplier = sideTopMultiplier;
            else if(inSection(angle, "side-bottom"))
                currMultiplier = sideBottomMultiplier;
            else if(inSection(angle, "bottom"))
                currMultiplier = bottomMultiplier;

            currMultiplier = Math.max(currMultiplier, 1);

            for(int j = 0; j < currMultiplier - 1; j++)
                downswingPoints.add(i, downswingPoints.get(i));

            i += currMultiplier - 1;
        }
    }

    // returns if buffer is full
    private boolean handleBuffer(Mat[] processedBuffer, Mat curr, Mat prev) {
        processedBuffer[0] = processedBuffer[1];
        processedBuffer[1] = preprocess(curr, prev);

        return processedBuffer[0] != null;
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    private double length(double[] line) {
        return Math.sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
    }

    /**
     * Finds points within a radius of the y median. The radius is a percent value in terms of the median value.
     * @param points Points to find the points around the y median of
     * @param radius Allow points within this radius of the median (pixels)
     * @return Returns an array of the points
     */
    private Point[] withinMedian(ArrayList<Point> points, int radius) {
        if(points.size() == 0)
            return null;

        Point[] sorted = points.toArray(new Point[points.size()]);
        Arrays.sort(sorted, (o1, o2) -> {
            Point p1 = (Point) o1;
            Point p2 = (Point) o2;

            if(p1.y < p2.y)
                return -1;

            if(p1.y > p2.y)
                return 1;

            return 0;
        });

        Point median = sorted[sorted.length / 2];
        int start = closestYIndex(sorted, median.y + radius, 0, sorted.length - 1);
        int end = closestYIndex(sorted, median.y - radius, start, sorted.length - 1);

        Point[] within = new Point[end - start + 1];
        for(int i = start; i <= end; i++)
            within[i - start] = sorted[i];

        return within;
    }

    /**
     * Returns the index of the point where the given y value is the closest to the one in the array
     * @param points The points to find the index of
     * @param y The y value to find the closest to
     * @param start The lower bound index for the search
     * @param end The upper bound index for the search
     * @return Returns the index of the closest y value of all the points in the array
     */
    private int closestYIndex(Point[] points, double y, int start, int end) {
        while(start < end) {
            int mid = start + (end - start) / 2;

            if(points[mid].y <= y)
                start = mid + 1;
            else
                end = mid;
        }

        return start;
    }

    /**
     * Returns where the given value should be inserted in the orderd list
     * @param list The orderd list
     * @param value The value to find the insertion index
     * @param start The lower bound for the search
     * @param end The upper bound for the search
     * @return Returns the given index where the value should be placed so that the list stays ordered
     */
    private int getInsertIndex(ArrayList<Integer> list, double value, int start, int end) {
        while(start < end) {
            int mid = start + (end - start) / 2;

            if(list.get(mid) <= value)
                start = mid + 1;
            else
                end = mid;
        }

        return start;
    }

    /**
     * Gets if the given angle (radians) is within the given portion of the swing
     * @param angle The angle see the section of [-pi, pi] (angle from atan2)
     * @param section The section of the swing ("top", "side", "bottom")
     * @return Returns if the angle is in the given section
     */
    public static boolean inSection(double angle, String section) {
        switch (section) {
            case "top":
                return angle >= 0 && angle < 2.5;//(3 * Math.PI / 4);
            case "side-top":
//                return (angle >= (3 * Math.PI / 4) && angle <= Math.PI) || (angle >= -Math.PI && angle <= (-3 * Math.PI / 4));
                return angle >= 2.5 && angle <= Math.PI;
            case "side-bottom":
                return angle >= -Math.PI && angle <= (-3 * Math.PI / 4);
            case "bottom":
                return angle > (-3 * Math.PI / 4) && angle <= 0;
        }

        return false;
    }
}
