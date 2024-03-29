import data.Circle;
import data.SwingFit;
import data.TimedPoint;
import data.testData.SwingPoints;
import org.opencv.core.*;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.Features2d;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.opencv.xfeatures2d.SURF;
import util.BezierFit;

import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Class for getting keypoint tracking from the given video.
 */
public class KeypointTracking {
    public static int FLAG_DISPLAY_INFO = 1 << 0;

    private int flags;

    private String fileName;

    private ArrayList<TimedPoint> points;
    private ArrayList<TimedPoint> backswingPoints;
    private ArrayList<TimedPoint> downswingPoints;

    private ArrayList<TimedPoint> medianPoints;
    private ArrayList<TimedPoint> medianPointsDownswing;
    private ArrayList<Double> blurredY;
    private ArrayList<Double> blurredX;

    private Point downswingPoint;
    private int downswingFrame;
    private int downswingEndFrame;
    private int downswingFrameYVal;
    private int downswingFrameXVal;

    private Mat trackingLines;
    private Point referencePoint;
    private double[] clubLine;

    private Circle ball;

    private int frame;
    private int endFrame;
    private int fps;

    private Mat tracer;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public KeypointTracking(String fileName, int startFrame, int endFrame, int flags) {
        this.flags = flags;

        this.fileName = fileName;

        points = new ArrayList<TimedPoint>();
        backswingPoints = new ArrayList<TimedPoint>();
        downswingPoints = new ArrayList<TimedPoint>();

        medianPoints = new ArrayList<TimedPoint>();
        medianPointsDownswing = new ArrayList<TimedPoint>();
        blurredY = new ArrayList<Double>();
        blurredX = new ArrayList<Double>();

        downswingPoint = null;
        downswingFrame = 0;
        downswingEndFrame = 0;
        downswingFrameYVal = 0;
        downswingFrameXVal = 0;

        trackingLines = new Mat();
        referencePoint = null;
        clubLine = new double[4];

        ball = null;

        frame = startFrame;
        this.endFrame = endFrame;
        fps = 0;

        tracer = new Mat();

        findKeyPoints();
    }

    /**
     * Returns the best fit path and swing for the downswing
     * @return Returns a SwingFit object that represents the Bezier curves for both the path and the speed
     */
    public SwingFit fit() {
        BezierFit pathFit = fitPath();
        BezierFit speedFit = fitSpeed(pathFit);

        return new SwingFit(pathFit, speedFit);
    }

    /**
     * Returns a best fit Bezier curve for the downswing
     * @return Returns a BezierFit representing the curve of best fit for the path of the downswing
     */
    public BezierFit fitPath() {
//        ArrayList<TimedPoint> medianPointsDownswing = SwingPoints.Tiger.medianPointsDownswing;
//        ArrayList<TimedPoint> downswingPoints = SwingPoints.Tiger.downwingPoints;
//        Point referencePoint = SwingPoints.Tiger.referencePoint;
//        tracer = Mat.zeros(new Size(790, 720), CvType.CV_8UC3);

        long start = System.currentTimeMillis();
        BezierFit fit = BezierFit.RANSACRecursive(medianPointsDownswing, 7, medianPointsDownswing.size() / 2, 10, (int) 1e5);
        System.out.println("Path Fit Time: " + ((System.currentTimeMillis() - start) / 1000.0));

        if(flag(FLAG_DISPLAY_INFO)) {
//            tracer = Mat.zeros(trackingLines.size(), CvType.CV_8UC3);

            Point[] points = fit.getPoints(0.01);

            for(Point p : downswingPoints)
                Imgproc.circle(tracer, p, 3, new Scalar(255, 255, 255), -1);

            for(int i = 1; i < points.length; i++)
                Imgproc.line(tracer, points[i - 1], points[i], new Scalar(0, 255, 0));

            Imgproc.circle(tracer, referencePoint, 3, new Scalar(0, 255, 255), 1);

            HighGui.imshow("Tracer", tracer);

            System.out.println("Press any key to continue...");

            HighGui.waitKey(0);
        }

        return fit;
    }

    /**
     * Returns a best fit Bezer curve for the speed of the swing (x: frame, y: t [ 0,1])
     * @param pathFit The BezierFit for the path of the downswing
     * @return Returns a BezierFit representing the curve of best fit for speed of the downswing
     */
    public BezierFit fitSpeed(BezierFit pathFit) {
//        ArrayList<TimedPoint> medianPointsDownswing = SwingPoints.Tiger.medianPointsDownswing;

        ArrayList<Point> rawFrameToT = new ArrayList<Point>(); // x: frame, y: t

        ArrayList<TimedPoint> filtered = new ArrayList<TimedPoint>();

        // aren't these just downswing points?

        if(medianPointsDownswing.size() > 0)
            filtered.add(medianPointsDownswing.get(0));

        for(int i = 1; i < medianPointsDownswing.size(); i++) {
            if(!((Point) medianPointsDownswing.get(i)).equals((Point) medianPointsDownswing.get(i - 1)))
                filtered.add(medianPointsDownswing.get(i));
        }

        for(TimedPoint p : filtered) {
            double[] min = pathFit.getMinDistance(p, 0.001);
            double dist = min[0];
            double t = min[1];

            if (dist < 10)
                rawFrameToT.add(new Point(p.frame, t));
        }

        BezierFit speedFit = new BezierFit(rawFrameToT, 4);

        Mat speed = Mat.zeros(500, (int) ((rawFrameToT.get(rawFrameToT.size() - 1).x - rawFrameToT.get(0).x) * 5), CvType.CV_8UC3);
        Point[] speedPoints = speedFit.getPoints(0.01);

        Point first = rawFrameToT.get(0);

        for(Point p : rawFrameToT)
            Imgproc.circle(speed, new Point((p.x - first.x) * 5, p.y * 500), 3, new Scalar(255, 255, 255));

        for(int i = 1; i < speedPoints.length; i++) {
            Point p1 = speedPoints[i];
            Point p2 = speedPoints[i - 1];

            Imgproc.line(speed, new Point((p1.x - first.x) * 5, p1.y * 500), new Point((p2.x - first.x) * 5, p2.y * 500), new Scalar(255, 100, 255));
        }

        System.out.println("Showing speed fit: press any key to continue...");
        HighGui.imshow("Speed fit", speed);
        HighGui.waitKey(0);

        return speedFit;
    }

    /**
     * Finds the KeyPoints in the video and puts them in the ArrayLists
     */
    private void findKeyPoints() {
        VideoCapture capture = new VideoCapture(fileName);
        fps = (int) capture.get(Videoio.CAP_PROP_FPS);

        capture.set(Videoio.CAP_PROP_POS_FRAMES, frame);

        Mat prev = new Mat();
        Mat curr = new Mat();
        Mat[] processedBuffer = new Mat[2];

        // display info
        Mat yGraph = new Mat();
        Mat xGraph = new Mat();
        Mat segmentation = new Mat();

        boolean ok = capture.read(curr);

        trackingLines = new Mat(curr.size(), curr.type());

        while(ok && frame < endFrame) {
            frame++;

            System.out.println("Frame: " + frame);

            if(!prev.empty() && !curr.empty()) {
                boolean bufferFull = handleBuffer(processedBuffer, curr, prev);

                if(bufferFull) {
                    Mat trackInfo = trackClub(processedBuffer[1], processedBuffer[0]);

                    if(flag(FLAG_DISPLAY_INFO)) {
                        // move logic to separate function?
                        if(ball != null)
                            Imgproc.circle(trackInfo, ball.getCenter(), (int) ball.getRadius(), new Scalar(0, 100, 100), 1);

                        HighGui.imshow("Preprocess", preprocess(curr, prev));
                        HighGui.imshow("Track", trackInfo);
                    }
                }

                if(ball == null)
                    findBall(curr);

                HighGui.waitKey(1000 / fps);
            }

            prev = curr.clone();
            ok = capture.read(curr);

            System.out.println("OK: " + ok);
        }

        // post processing
        if(blurredY.size() == 0) {
            postprocess();

            if(flag(FLAG_DISPLAY_INFO)) {
                System.out.println("ReferencePoint: " + referencePoint);

                System.out.println("Downswing points:");
                for(TimedPoint p : downswingPoints)
                    System.out.println("new TimedPoint(" + p.frame + ", new Point(" + p.x + ", " + p.y + ")),");

                System.out.println("Median points downswing:");
                for(TimedPoint p : medianPointsDownswing)
                    System.out.println("new TimedPoint(" + p.frame + ", new Point(" + p.x + ", " + p.y + ")),");

                yGraph = Mat.zeros(curr.rows(), blurredY.size(), CvType.CV_8UC3);
                for(int i = 0; i < blurredY.size(); i++)
                    Imgproc.circle(yGraph, new Point(i, blurredY.get(i)), 3, new Scalar(255, 255, 70), -1);

                xGraph = Mat.zeros(curr.rows(), blurredX.size(), CvType.CV_8UC3);
                for(int i = 0; i < blurredX.size(); i++)
                    Imgproc.circle(xGraph, new Point(i, blurredX.get(i)), 3, new Scalar(255, 70, 255), -1);

                Imgproc.circle(yGraph, new Point(downswingFrame - medianPoints.get(0).frame, downswingFrameYVal), 5, new Scalar(255, 255, 255), -1);
                Imgproc.line(trackingLines, new Point(0, downswingFrameYVal), new Point(curr.cols() - 1, downswingFrameYVal), new Scalar(0, 255, 0));
                Imgproc.putText(yGraph, downswingFrame + "", new Point(0, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                Imgproc.putText(yGraph, blurredY.size() + "", new Point(0, 200), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                HighGui.imshow("YGraph", yGraph);

                Imgproc.circle(xGraph, new Point(downswingFrame - medianPoints.get(0).frame, downswingFrameXVal), 5, new Scalar(255, 255, 255), -1);
                Imgproc.putText(xGraph, downswingFrame + "", new Point(0, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                Imgproc.putText(xGraph, blurredX.size() + "", new Point(0, 200), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255));
                HighGui.imshow("XGraph", xGraph);

                Imgproc.circle(trackingLines, downswingPoint, 3, new Scalar(255, 70, 255), -1);

                segmentation = Mat.zeros(curr.size(), CvType.CV_8UC3);
                for(TimedPoint p : backswingPoints)
                    Imgproc.circle(segmentation, p, 3, new Scalar(255, 0, 0), -1);

                for(TimedPoint p : downswingPoints)
                    Imgproc.circle(segmentation, p, 3, new Scalar(100, 255, 255), -1);
            }
        }

        // wrong order? this block shoudl get called continuously after, but the one above only once <-- check if null?
        if(flag(FLAG_DISPLAY_INFO)) {
            HighGui.imshow("Tracking Lines", trackingLines);
            HighGui.imshow("YGraph", yGraph);
            HighGui.imshow("XGraph", xGraph);
            HighGui.imshow("Segmentation", segmentation);

//            HighGui.waitKey(0);
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

        // TODO: ADD A THRESHHOLD?

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

        Mat houghLinesDest = new Mat();
        Imgproc.cvtColor(res, houghLinesDest, Imgproc.COLOR_GRAY2BGR);

        double[] l = null;
        double largest = 0;

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

        if(flag(FLAG_DISPLAY_INFO))
            HighGui.imshow("Hough Lines", houghLinesDest);

        Imgproc.cvtColor(res, res, Imgproc.COLOR_GRAY2BGR);

        return res;
    }

    private Mat trackClub(Mat curr, Mat prev) {
        // detect keypoints

        SURF kpFinder = SURF.create(1500, 8, 5, true, false);

        MatOfKeyPoint currKP = new MatOfKeyPoint();
        Mat currDescriptors = new Mat();

        kpFinder.detect(curr, currKP);
        kpFinder.compute(curr, currKP, currDescriptors);

        MatOfKeyPoint prevKP = new MatOfKeyPoint();
        Mat prevDescriptors = new Mat();

        kpFinder.detect(prev, prevKP);
        kpFinder.compute(prev, prevKP, prevDescriptors);

        Mat key = new Mat();
        Features2d.drawKeypoints(curr, currKP, key, Scalar.all(-1), Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS);

        // match kepoints

        if(currDescriptors.empty() || prevDescriptors.empty()) {
            Mat out = new Mat();

            Core.add(curr, trackingLines, out);

            return out;
        }

        BFMatcher matcher = BFMatcher.create(Core.NORM_L2, true);

        MatOfDMatch matches = new MatOfDMatch();
        matcher.match(currDescriptors, prevDescriptors, matches, new Mat()); // query, train

        // filter matches

        KeyPoint[] currKPArray = currKP.toArray();
        KeyPoint[] prevKPArray = prevKP.toArray();

        // Only use points within 1 standard deviation of the median?
        ArrayList<TimedPoint> filteredPoints = new ArrayList<TimedPoint>();

        if(clubLine != null) {
            for (DMatch m : matches.toArray()) {
                Point p1 = currKPArray[m.queryIdx].pt;
                Point p2 = prevKPArray[m.trainIdx].pt;

                if(Math.abs(p1.x - p2.x) < 20 && Math.abs(p1.y - p2.y) < 20 &&
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

                    if(headDist < gripDist)
                        filteredPoints.add(new TimedPoint(frame, p1));
                }
            }

            // further filters keypoints by only including those that lie within a certain
            // distance from the median
            if(filteredPoints.size() != 0) {
                TimedPoint[] withinMedian = withinMedian(filteredPoints, 5);
                points.addAll(Arrays.asList(withinMedian));

                for(TimedPoint p : withinMedian)
                    Imgproc.circle(trackingLines, p, 3, new Scalar(255, 0, 0), -1);

                TimedPoint mid = withinMedian[withinMedian.length / 2];
                medianPoints.add(new TimedPoint(frame, mid));
            }
        } else if(medianPoints.size() != 0) {
            medianPoints.add(new TimedPoint(frame, medianPoints.get(medianPoints.size() - 1)));
        }

        Mat out = new Mat();

        Core.add(curr, trackingLines, out);

        if(clubLine != null)
            Imgproc.rectangle(out, new Point(clubLine[0], clubLine[1]), new Point(clubLine[2], clubLine[3]), new Scalar(0, 100, 0));

        if(referencePoint != null)
            Imgproc.circle(out, referencePoint, 5, new Scalar(255, 0, 255));

        return out;
    }

    /**
     * Finds the ball using the Hough Circle Transform. Only looks for the ball below the reference point
     * so, if the reference point is null, the ball will not be looked for.
     * @param curr The current frame
     */
    private void findBall(Mat curr) {
        if(referencePoint == null || medianPoints.size() == 0)
            return;

        Mat gray = new Mat();
        Imgproc.cvtColor(curr, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.medianBlur(gray, gray, 5);

        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, (double) gray.rows() / 16, 260, 6,
                1, 15);

        ArrayList<Circle> guesses = new ArrayList<Circle>();
        for(int i = 0; i < circles.cols(); i++) {
            double[] c = circles.get(0, i);

            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            int radius = (int) Math.round(c[2]);

            if(center.y > referencePoint.y)
                guesses.add(new Circle(center, radius));
        }

        double minDist = Integer.MAX_VALUE;
        Point estimation = new Point(referencePoint.x, medianPoints.get(0).y);
        for(Circle c : guesses) {
            // find point closest to initial club keypoint
            double dist = distance(estimation, c.getCenter());

            if(dist < minDist) {
                ball = c;

                minDist = dist;
            }
        }

        if(ball != null) {
            System.out.println("Ball: " + ball + ", " + fps);

            Mat out = curr.clone();
            for(Circle c : guesses)
                Imgproc.circle(out, c.getCenter(), (int) c.getRadius(), new Scalar(255, 100, 100));

            Imgproc.circle(out, ball.getCenter(), (int) ball.getRadius(), new Scalar(100, 255, 100));
        }
    }

    private void postprocess() {
        // plot y values in array in accordance to time and blur
        for(TimedPoint tp : medianPoints) {
            blurredX.add(tp.x);
            blurredY.add(tp.y);
        }

        blur(blurredX, 31, 15);
        blur(blurredY, 31, 15);

        // find min y val to find downswing frame
//        for(int i = 3; i < blurredY.size() - 3; i++) {
//            double curr = blurredY.get(i);
//
//            if(blurredY.get(i - 2) < curr && blurredY.get(i - 1) < curr &&
//                    blurredY.get(i + 1) < curr && blurredY.get(i + 2) < curr) {
//                downswingFrame = (int) medianPoints.get(i).frame;
//                downswingFrameYVal = (int) curr;
//
//                i = blurredY.size();
//            }
//        }


        // find downswing start and end frames

        int idx = 0;

        // find point of inflection to find downswing end frame
        boolean check = false;
        for(int i = 2; i < blurredY.size() - 1; i++) {
            double prevV = blurredY.get(i - 1) - blurredY.get(i - 2);
            double v = blurredY.get(i) - blurredY.get(i - 1);
            double nextV = blurredY.get(i + 1) - blurredY.get(i);

            if(v > 0)
                check = true;

            if(check) {
                if(v - prevV <= 0 && nextV - v >= 0) {
                    downswingFrame = (int) medianPoints.get(i).frame;
                    downswingFrameYVal = (int) (double) blurredY.get(i);

                    idx = i + 1;

                    i = blurredY.size();
                }
            }
        }

        downswingEndFrame = (int) medianPoints.get(medianPoints.size() - 1).frame;

        for(int i = idx; i < blurredY.size()- 1; i++) {
            double prev = blurredY.get(i - 1);
            double curr = blurredY.get(i);
            double next = blurredY.get(i + 1);

            if(curr <= prev && curr <= next)
                downswingEndFrame = (int) medianPoints.get(i).frame;
        }

        System.out.println("DOWNSWING END FRAME: " + downswingEndFrame);

//        for(int i = 3; i < blurredX.size() - 3; i++) {
//            double curr = blurredX.get(i);
//
//            if(blurredX.get(i - 2) < curr && blurredX.get(i - 1) < curr &&
//                    blurredX.get(i + 1) < curr && blurredX.get(i + 2) < curr) {
////                downswingFrame = keyPointBasedTime.get(i);
//                downswingFrameXVal = (int) curr;
//
//                i = blurredX.size();
//            }
//        }

        for(int i = 0; i < points.size(); i++) {
            TimedPoint p = points.get(i);

            if(p.frame < downswingFrame)
                backswingPoints.add(p);
            else
                downswingPoints.add(p);
        }

        downswingPoint = downswingPoints.get(0);

        // remove first 20% of downswing points b/c lots of noise
        int n = (int) (downswingPoints.size() * 0.2);
        for(int i = 0; i < n; i++)
            downswingPoints.remove(0);

        for(int i = 0; i < medianPoints.size(); i++) {
            TimedPoint p = medianPoints.get(i);

            if(p.frame >= downswingPoints.get(0).frame && p.frame <= downswingPoints.get(downswingPoints.size() - 1).frame)
                medianPointsDownswing.add(p);
        }

        // get rid of points in the middle that probably belong to the club
        double maxY = referencePoint.y; // bottom most
        double minY = referencePoint.y; // top most

        double maxX = referencePoint.x; // right most
        double minX = referencePoint.x; // left most

        for(Point p : medianPointsDownswing) {
            if(p.y > maxY)
                maxY = p.y;

            if(p.y < minY)
                minY = p.y;

            if(p.x > maxX)
                maxX = p.x;

            if(p.x < minX)
                minX = p.x;
        }

        double radius = Math.min(maxY - minY, maxX - minX) / 2;
        double radiusPercentage = 0.4;
        Point center = new Point(referencePoint.x, minY + (maxY - minY) / 2);

        for(int i = medianPointsDownswing.size() - 1; i >= 0; i--) {
            double dist = distance(center, medianPointsDownswing.get(i));

            if(dist < radiusPercentage * radius)
                medianPointsDownswing.remove(i);
        }

        tracer = Mat.zeros(trackingLines.size(), CvType.CV_8UC3);
        Imgproc.circle(tracer, center, 3, new Scalar(41, 84, 97), 1);
        Imgproc.circle(tracer, center, (int) (radiusPercentage * radius), new Scalar(41, 84, 97), 1);

        // increase densities of sections
        int top = 0;
        int sideTop = 0;
        int sideBottom = 0;
        int bottom = 0;

        for(int i = 0; i < downswingPoints.size(); i++) {
            TimedPoint p = downswingPoints.get(i);
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
        int bottomMultiplier = (int) ((top * 0.4) / bottom); // 0.3

//        System.out.println("MULTIPLIER (SIDE TOP): " + sideTopMultiplier);
//        System.out.println("MULTIPLIER (SIDE BOTTOM): " + sideBottomMultiplier);
//        System.out.println("MULTIPLIER (BOTTOM): " + bottomMultiplier);
//        System.out.println("TOP: " + top);
//        System.out.println("SIDE TOP: " + sideTop);
//        System.out.println("SIDE BOTTOM: " + sideBottom);
//        System.out.println("BOTTOM: " + bottom);

        for(int i = 0; i < downswingPoints.size(); i++) {
            TimedPoint p = downswingPoints.get(i);
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

        System.out.println("POST: " + medianPointsDownswing.size());
    }

    /**
     * Handles updating the Mats in the buffer
     * @param processedBuffer The buffer
     * @param curr The Mat from the current frame
     * @param prev The Mat from the previous frame
     * @return Returns whether or not the buffer is full
     */
    private boolean handleBuffer(Mat[] processedBuffer, Mat curr, Mat prev) {
        processedBuffer[0] = processedBuffer[1];
        processedBuffer[1] = preprocess(curr, prev);

        return processedBuffer[0] != null;
    }

    /**
     * Finds the distance from one point to another
     * @param p1 The first point
     * @param p2 The second point
     * @return The distance (px) from the first point to the second point
     */
    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * Finds the length of a given line given the endpoints
     * @param line The endpoints of the line in the form of [x1, y1, x2, y2]
     * @return Returns the length of the line (px)
     */
    private double length(double[] line) {
        return Math.sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
    }

    /**
     * Finds points within a radius of the y median. The radius is a percent value in terms of the median value.
     * @param points Points to find the points around the y median of
     * @param radius Allow points within this radius of the median (pixels)
     * @return Returns an array of the points
     */
    private TimedPoint[] withinMedian(ArrayList<TimedPoint> points, int radius) {
        if(points.size() == 0)
            return null;

        TimedPoint[] sorted = points.toArray(new TimedPoint[points.size()]);
        Arrays.sort(sorted, (o1, o2) -> {
            Point p1 = ((TimedPoint) o1);
            Point p2 = ((TimedPoint) o2);

            if(p1.y < p2.y)
                return -1;

            if(p1.y > p2.y)
                return 1;

            return 0;
        });

        TimedPoint median = sorted[sorted.length / 2];
        int start = closestYIndex(sorted, median.y + radius, 0, sorted.length - 1);
        int end = closestYIndex(sorted, median.y - radius, start, sorted.length - 1);

        TimedPoint[] within = new TimedPoint[end - start + 1];
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
    private int closestYIndex(TimedPoint[] points, double y, int start, int end) {
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
     * Returns where the given value should be inserted in the orderd list of TimedPoints in accordance to the time
     * @param list The orderd list
     * @param value The value to find the insertion index
     * @param start The lower bound for the search
     * @param end The upper bound for the search
     * @return Returns the given index where the value should be placed so that the list stays ordered
     */
    private int getInsertIndex(ArrayList<TimedPoint> list, double value, int start, int end) {
        while(start < end) {
            int mid = start + (end - start) / 2;

            if(list.get(mid).frame <= value)
                start = mid + 1;
            else
                end = mid;
        }

        return start;
    }

    /**
     * Blurs the values in an ArrayList
     * @param values The values to blur
     * @param kernelSize The size of the Gaussian blur kernel
     * @param numTimes Number of times to run the blur
     */
    private void blur(ArrayList<Double> values, int kernelSize, int numTimes) {
        for(int i = 0; i < numTimes; i++) {
            Mat blurredMat = new Mat(values.size(), 1, CvType.CV_32F);

            double[] buffer = new double[values.size()];
            for(int j = 0; j < buffer.length; j++)
                buffer[j] = values.get(j);

            blurredMat.put(0, 0, buffer);

            Imgproc.GaussianBlur(blurredMat, blurredMat, new Size(1, kernelSize), 0, 0);

            for(int j = 0; j < blurredMat.rows(); j++)
                values.set(j, blurredMat.get(j, 0)[0]);
        }
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

    /**
     * Checks to see if the flag has been set
     * @param flag The flag constant
     * @return Returns true if the flag was passed in
     */
    private boolean flag(int flag) {
        return (flag & flags) == flag;
    }

    public int getDownswingFrame() {
        return downswingFrame;
    }

    public Circle getBall() {
        return ball;
    }

    public int getFPS() {
        return fps;
    }
}
