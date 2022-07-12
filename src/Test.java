import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.Objdetect;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.opencv.xfeatures2d.SURF;
import util.BezierFit;
import util.RandColor;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

public class Test {

    // TODo: SPACING WAS THE CULPRIT

    enum Section {
        BACKSWING,
        DOWNSWING,
    }

    private Mat trackingLines;
    private double[] clubLine;
    private Point referencePoint;

    ArrayList<Point> trackPoints;

    private Section section;
    private ArrayList<Double> clubPos;
    private int dirChange;

//    private int spacing;

    private PrintWriter writer;

    private int frame;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public Test() {
        trackingLines = new Mat();
        clubLine = new double[4];
        referencePoint = null;

        trackPoints = new ArrayList<Point>();

        section = Section.BACKSWING;
        clubPos = new ArrayList<Double>();
        dirChange = 0;

        try {
            writer = new PrintWriter(new FileWriter("points.txt"));
        } catch(IOException e) {
            System.out.println("Unable to open file: points.txt");
        }

        frame = 0;
    }

    public static void main(String[] args) {
        Test test = new Test();
        test.run();
    }

    private void run() {
        VideoCapture capture = new VideoCapture("src/res/tigerdriver.mp4");
        double fps = 120;//capture.get(Videoio.CAP_PROP_FPS);

        Mat prev = new Mat();
        Mat curr = new Mat();

        Mat[] processedBuffer = new Mat[2];
        Mat tracer = new Mat();

        while(true) {
            boolean ok = capture.read(curr);

            frame++;

            if(frame == 515) {
                BezierFit fit = new BezierFit(trackPoints,  10);
                Point[] points = fit.getPoints(0.1);

                tracer = trackingLines.clone();
                for(int i = 1; i < points.length; i++) {
                    Imgproc.line(tracer, points[i - 1], points[i], new Scalar(0, 255, 0));
                    System.out.println("line: " + points[i - 1] + " -> " + points[i]);
                }

                for(Point p : trackPoints) {
                    System.out.println("new Point(" + p.x + ", " + p.y + "), ");
                }
            }

            System.out.println("frame: " + frame);

            if(frame >= 515)
                HighGui.imshow("Tracer", tracer);

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
                   HighGui.imshow("Track", res);
               }

               HighGui.waitKey((int) ((1 / fps) * 1000));
            }

            prev = curr.clone();
        }
    }

    // returns if buffer is full
    private boolean handleBuffer(Mat[] processedBuffer, Mat curr, Mat prev) {
        processedBuffer[0] = processedBuffer[1];
        processedBuffer[1] = preprocess(curr, prev);

        return processedBuffer[0] != null;
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

        Mat dest = new Mat(res.size(), res.type());
        Core.addWeighted(res, 1.5, dest, -0.5, 0, dest);

        Imgproc.Canny(res.clone(), res, 80, 200);

        // probabilistic Hough line transform
        Mat lines = new Mat();
        Imgproc.HoughLinesP(res, lines, 1, Math.PI / 180, 80, 50, 30);

//        System.out.println(lines.dump());

        Mat houghLinesDest = new Mat();
        Imgproc.cvtColor(res, houghLinesDest, Imgproc.COLOR_GRAY2BGR);

        Mat houghLinesFilterd = new Mat();

        HashSet<Double> okSlopes = new HashSet<Double>();

        for(int i = 0; i < lines.rows(); i++) {
            double[] l1 = lines.get(i, 0);
            double slope1 = (l1[3] - l1[1]) / (l1[2] - l1[0]);

            if(okSlopes.contains(slope1))
                i++;

            for(int j = 0; j < lines.rows(); j++) {
                double[] l2 = lines.get(j, 0);
                double slope2 = (l2[3] - l2[1]) / (l2[2] - l2[0]);

                if(Math.min(slope1, slope2) / Math.min(slope1, slope2) <= 0.95) {
                    houghLinesFilterd.put(houghLinesFilterd.rows(), 0 , l1);

                    okSlopes.add(slope1);

                    j = lines.rows();
                }
            }
        }

        // use median distance insteads of random?

        double[] l = null;
        double largest = 0;

        System.out.println(lines.rows());

        for(int i = 0; i < lines.rows(); i++) {
            if(lines.get(i, 0).length == 0) {

                System.out.println("hi");
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

//        if(clubLine != null) {
//            if(referencePoint == null) {
//                if(clubLine[1] < clubLine[3])
//                    referencePoint = new Point(clubLine[0], clubLine[1]);
//                else
//                    referencePoint = new Point(clubLine[2], clubLine[3]);
//            } else {
//                double dist1 = distance(new Point(clubLine[0], clubLine[1]), referencePoint);
//                double dist2 = distance(new Point(clubLine[2], clubLine[3]), referencePoint);
//
//                if(dist1 < dist2)
//                    referencePoint = new Point(clubLine[0], clubLine[1]);
//                else
//                    referencePoint = new Point(clubLine[2], clubLine[3]);
//            }
//        }

        HighGui.imshow("Hough Lines", houghLinesDest);

//        // Hough line transform
//
//        Mat lines = new Mat();
//        Imgproc.HoughLines(res, lines, 1, Math.PI / 180, 150);
//
//        for(int i = 0; i < lines.rows(); i++) {
//            double rho = lines.get(i, 0)[0];
//            double theta = lines.get(i, 0)[1];
//
//            double a = Math.cos(theta);
//            double b = Math.sin(theta);
//
//            double x0 = a * rho;
//            double y0 = b * rho;
//
//            Point p1 = new Point(Math.round(x0 + 1000 * -b), Math.round(y0 + 1000 * a));
//            Point p2 = new Point(Math.round(x0 - 1000 * -b), Math.round(y0 - 1000 * a));
//
//            Imgproc.line(curr, p1, p2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
//        }

//        Core.absdiff(curr, prev, res);
//
//        byte[] conv = new byte[] {
//          -1, -1, -1,
//          -1, 9, -1,
//          -1, -1, -1
//        };
//        ByteBuffer buffer = ByteBuffer.wrap(conv);
//        System.out.println(Arrays.toString(buffer.array()));
//        Mat kernel = new Mat(3, 3, CvType.CV_32F, ByteBuffer.wrap(conv));
//        Imgproc.filter2D(res, res, -1, kernel);

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

//        LinkedList<DMatch> goodMatches = new LinkedList<DMatch>();

        System.out.println("club line: " + Arrays.toString(clubLine));

        // TODO: CHANGE TO COMPARING TO THE KEYPOINT FROM 2 TIMES AGO

        // find median x val of matches to find probable club pos to use in determining swing section

        KeyPoint[] currKPArray = currKP.toArray();
        KeyPoint[] prevKPArray = prevKP.toArray();

//        ArrayList<Double> xVals = new ArrayList<Double>();
//        for(DMatch m : matches.toArray())
//            xVals.add(currKPArray[m.queryIdx].pt.x);
//
//        clubPos.add(median(xVals));
//
//        Scalar color = new Scalar(255, 255, 0);
//
//        if(clubPos.size() >= 7) {
//            clubPos.remove(0);
//
//            int numMonotonic = 0;
//            for(int i = 0; i < clubPos.size() - 1; i++) {
////                if(section == Section.BACKSWING) {
////                    if(clubPos.get(i))
////                } else if(section == Section.DOWNSWING) {
////
////                }
//
//                if(clubPos.get(i) < clubPos.get(i + 1))
//                    numMonotonic++;
//                else
//                    numMonotonic--;
//            }
//
//            System.out.println("nm: " + numMonotonic);
//
//            if(numMonotonic > 0)
//                color = new Scalar(0, 255, 0); // right
//            else if(numMonotonic < 0)
//                color = new Scalar(0, 0, 255); // left
//
//            System.out.println("color: " + color.toString());
//        }
//
//        int size = 100;
//        if(clubPos.size() > size) {
//            int threshold = (int) (size * 0.9);
//
//            int num = 0;
//            for(int i = size; i < clubPos.size(); i++) {
//                if(dirChange == 0) {
//
//                }
//            }
//        }

        // filter matches

        if(clubLine != null) {
//            for(DMatch m : matches.toArray()) {
//            }
//
//            DMatch m = (DMatch) median(new ArrayList<Object>(Arrays.asList(matches.toArray())), (o1, o2) -> {
//               DMatch m1 = (DMatch) o1;
//               DMatch m2 = (DMatch) o2;
//
//               if(currKPArray[m1.queryIdx].pt.x > currKPArray[m2.queryIdx].pt.x)
//                   return 1;
//               else if(currKPArray[m1.queryIdx].pt.x < currKPArray[m2.queryIdx].pt.x)
//                   return -1;
//               else
//                   return 0;
//            });

            for (DMatch m : matches.toArray()) {
                Point p1 = currKPArray[m.queryIdx].pt;
                Point p2 = prevKPArray[m.trainIdx].pt;

                if (Math.abs(p1.x - p2.x) < 20 && Math.abs(p1.y - p2.y) < 20 &&
                        !(p1.x > Math.min(clubLine[0], clubLine[2]) && p1.x < Math.max(clubLine[0], clubLine[2]) &&
                                p1.y > Math.min(clubLine[1], clubLine[3]) && p1.y < Math.max(clubLine[1], clubLine[3]))) {

//                    goodMatches.add(m);

                    System.out.println(m.distance);

//                    double dist1 = distance(new Point(clubLine[0], clubLine[1]), p1);
//                    double dist2 = distance(new Point(clubLine[2], clubLine[3]), p1);

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
                        Imgproc.line(trackingLines, p1, p2, new Scalar(255, 0, 0), 1);

                        trackPoints.add(p1);

//                        writer.write("[" + p1.x + ", " + p1.y + "],\n");
                    }
//                    else
//                        Imgproc.line(trackingLines, p1, p2, new Scalar(0, 0, 100), 1);
                }
            }
        }
//        } else {
//            System.out.println("none");
//        }

//        MatOfDMatch goodMatchesMat = new MatOfDMatch();
//        goodMatchesMat.fromList(goodMatches);

//        LinkedList<Point> queryList = new LinkedList<Point>();
//        LinkedList<Point> trainList = new LinkedList<Point>();
//
//        DMatch[] matchesArray = matches.toArray();
//        for(int i = 0; i < matchesArray.length; i++) {
//            DMatch match = matchesArray[i];
//
//            int x1 = currKP.get
//        }

        // draw matches

//        Features2d.drawMatches(currOriginal, currKP, prevOriginal, prevKP, goodMatchesMat, out);
//
//        Imgproc.resize(out, out, new Size(out.width() / 2, out.height() / 2));

        Mat out = new Mat();

        Core.add(curr, trackingLines, out);

        if(clubLine != null)
            Imgproc.rectangle(out, new Point(clubLine[0], clubLine[1]), new Point(clubLine[2], clubLine[3]), new Scalar(0, 100, 0));

        if(referencePoint != null)
            Imgproc.circle(out, referencePoint, 5, new Scalar(255, 0, 255));

        return out;
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    private double length(double[] line) {
        return Math.sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
    }

    private double median(ArrayList<Double> vals) {
        if(vals.size() == 0)
            return 0;

        ArrayList<Double> ordered = new ArrayList<Double>();

        ordered.add(vals.get(0));

        for(int i = 0; i < vals.size(); i++) {
            double curr = vals.get(i);

            for(int j = 0; j < ordered.size(); j++) {
                if(ordered.get(j) > curr) {
                    ordered.add(j, curr);

                    j = ordered.size();
                }
            }
        }

        return ordered.get(ordered.size() / 2);
    }

    private Object median(ArrayList<Object> vals, Comparator<Object> comparator) {
        if(vals.size() == 0)
            return 0;

        ArrayList<Object> ordered = new ArrayList<Object>();

        ordered.add(vals.get(0));

        for(int i = 0; i < vals.size(); i++) {
            Object curr = vals.get(i);

            for(int j = 0; j < ordered.size(); j++) {
                if(comparator.compare(ordered.get(j), curr) > 0) {
                    ordered.add(j, curr);

                    j = ordered.size();
                }
            }
        }

        return ordered.get(ordered.size() / 2);
    }
}
