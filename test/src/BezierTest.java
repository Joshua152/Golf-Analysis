import data.TimedPoint;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import util.BezierFit;
import util.BezierRANSAC;

import java.io.*;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class BezierTest {
    private ArrayList<TimedPoint> data;

    public BezierTest(ArrayList<TimedPoint> data) {
        this.data = data;
    }

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        ArrayList<TimedPoint> tiger = new ArrayList<TimedPoint>(List.of(
                new TimedPoint(406.0, new Point(552.223876953125, 233.78353881835938)),
                new TimedPoint(407.0, new Point(567.7965087890625, 234.10720825195312)),
                new TimedPoint(408.0, new Point(314.27764892578125, 270.28668212890625)),
                new TimedPoint(409.0, new Point(562.7823486328125, 215.5616455078125)),
                new TimedPoint(411.0, new Point(569.5637817382812, 219.92489624023438)),
                new TimedPoint(412.0, new Point(567.3004760742188, 218.28335571289062)),
                new TimedPoint(413.0, new Point(309.89410400390625, 280.4914855957031)),
                new TimedPoint(414.0, new Point(564.6286010742188, 217.76089477539062)),
                new TimedPoint(415.0, new Point(287.36260986328125, 276.9635009765625)),
                new TimedPoint(416.0, new Point(560.09326171875, 226.0443115234375)),
                new TimedPoint(417.0, new Point(557.741455078125, 216.9669952392578)),
                new TimedPoint(418.0, new Point(547.0072021484375, 216.27186584472656)),
                new TimedPoint(420.0, new Point(549.7757568359375, 215.54684448242188)),
                new TimedPoint(421.0, new Point(543.8775634765625, 220.12818908691406)),
                new TimedPoint(422.0, new Point(539.9353637695312, 198.81858825683594)),
                new TimedPoint(423.0, new Point(538.634765625, 194.84942626953125)),
                new TimedPoint(424.0, new Point(525.7261962890625, 205.8837432861328)),
                new TimedPoint(425.0, new Point(528.0158081054688, 209.3490753173828)),
                new TimedPoint(426.0, new Point(516.1280517578125, 201.6374053955078)),
                new TimedPoint(428.0, new Point(507.71435546875, 211.9086456298828)),
                new TimedPoint(430.0, new Point(507.32952880859375, 198.89407348632812)),
                new TimedPoint(431.0, new Point(470.7920837402344, 203.55960083007812)),
                new TimedPoint(432.0, new Point(484.72808837890625, 193.50079345703125)),
                new TimedPoint(433.0, new Point(476.7325744628906, 188.44703674316406)),
                new TimedPoint(434.0, new Point(482.0361633300781, 184.67845153808594)),
                new TimedPoint(435.0, new Point(436.01141357421875, 321.5766296386719)),
                new TimedPoint(436.0, new Point(470.6485900878906, 190.6405792236328)),
                new TimedPoint(437.0, new Point(461.5006408691406, 191.01943969726562)),
                new TimedPoint(438.0, new Point(425.94775390625, 199.65834045410156)),
                new TimedPoint(439.0, new Point(445.8598327636719, 188.9493408203125)),
                new TimedPoint(440.0, new Point(411.6012268066406, 200.0704345703125)),
                new TimedPoint(441.0, new Point(429.38836669921875, 190.7281036376953)),
                new TimedPoint(442.0, new Point(419.9870910644531, 192.53384399414062)),
                new TimedPoint(443.0, new Point(404.9434814453125, 193.28514099121094)),
                new TimedPoint(444.0, new Point(401.0522766113281, 191.9495849609375)),
                new TimedPoint(445.0, new Point(385.9886474609375, 194.004150390625)),
                new TimedPoint(446.0, new Point(371.0223388671875, 201.6852264404297)),
                new TimedPoint(447.0, new Point(369.34832763671875, 194.94190979003906)),
                new TimedPoint(448.0, new Point(363.9318542480469, 195.62892150878906)),
                new TimedPoint(449.0, new Point(340.9764709472656, 199.89404296875)),
                new TimedPoint(450.0, new Point(338.25445556640625, 200.64593505859375)),
                new TimedPoint(451.0, new Point(318.08538818359375, 224.88243103027344)),
                new TimedPoint(452.0, new Point(290.94769287109375, 229.93740844726562)),
                new TimedPoint(453.0, new Point(294.9083251953125, 228.67247009277344)),
                new TimedPoint(454.0, new Point(275.91632080078125, 217.68861389160156)),
                new TimedPoint(455.0, new Point(255.06884765625, 266.0633544921875)),
                new TimedPoint(456.0, new Point(222.97975158691406, 231.82623291015625)),
                new TimedPoint(457.0, new Point(232.87335205078125, 249.2425537109375)),
                new TimedPoint(458.0, new Point(209.8844451904297, 252.89794921875)),
                new TimedPoint(459.0, new Point(197.291259765625, 249.7899169921875)),
                new TimedPoint(460.0, new Point(188.0839080810547, 253.89573669433594)),
                new TimedPoint(461.0, new Point(179.5569610595703, 275.51910400390625)),
                new TimedPoint(462.0, new Point(178.1430206298828, 284.14892578125)),
                new TimedPoint(463.0, new Point(161.63648986816406, 297.1679382324219)),
                new TimedPoint(463.0, new Point(161.63648986816406, 297.1679382324219)),
                new TimedPoint(463.0, new Point(161.63648986816406, 297.1679382324219)),
                new TimedPoint(464.0, new Point(153.85562133789062, 300.9812927246094)),
                new TimedPoint(464.0, new Point(153.85562133789062, 300.9812927246094)),
                new TimedPoint(464.0, new Point(153.85562133789062, 300.9812927246094)),
                new TimedPoint(465.0, new Point(134.9081573486328, 327.4660949707031)),
                new TimedPoint(465.0, new Point(134.9081573486328, 327.4660949707031)),
                new TimedPoint(465.0, new Point(134.9081573486328, 327.4660949707031)),
                new TimedPoint(466.0, new Point(122.98680114746094, 342.00396728515625)),
                new TimedPoint(466.0, new Point(122.98680114746094, 342.00396728515625)),
                new TimedPoint(466.0, new Point(122.98680114746094, 342.00396728515625)),
                new TimedPoint(467.0, new Point(131.77882385253906, 349.3769226074219)),
                new TimedPoint(467.0, new Point(131.77882385253906, 349.3769226074219)),
                new TimedPoint(467.0, new Point(131.77882385253906, 349.3769226074219)),
                new TimedPoint(468.0, new Point(124.6329116821289, 341.1186218261719)),
                new TimedPoint(468.0, new Point(124.6329116821289, 341.1186218261719)),
                new TimedPoint(468.0, new Point(124.6329116821289, 341.1186218261719)),
                new TimedPoint(469.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(469.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(469.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(487.0, new Point(307.8556213378906, 639.7841186523438)),
                new TimedPoint(487.0, new Point(307.8556213378906, 639.7841186523438)),
                new TimedPoint(487.0, new Point(307.8556213378906, 639.7841186523438)),
                new TimedPoint(488.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(488.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(488.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(493.0, new Point(443.2032775878906, 645.1068115234375)),
                new TimedPoint(493.0, new Point(443.2032775878906, 645.1068115234375)),
                new TimedPoint(493.0, new Point(443.2032775878906, 645.1068115234375)),
                new TimedPoint(494.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(494.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(494.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(500.0, new Point(602.7105102539062, 604.6414184570312)),
                new TimedPoint(500.0, new Point(602.7105102539062, 604.6414184570312)),
                new TimedPoint(500.0, new Point(602.7105102539062, 604.6414184570312))
        ));

        ArrayList<TimedPoint> medianPointsDownswing = new ArrayList<TimedPoint>(List.of(
                new TimedPoint(406.0, new Point(552.223876953125, 233.78353881835938)),
                new TimedPoint(407.0, new Point(567.7965087890625, 234.10720825195312)),
                new TimedPoint(408.0, new Point(314.27764892578125, 270.28668212890625)),
                new TimedPoint(409.0, new Point(562.7823486328125, 215.5616455078125)),
                new TimedPoint(411.0, new Point(569.5637817382812, 219.92489624023438)),
                new TimedPoint(412.0, new Point(567.3004760742188, 218.28335571289062)),
                new TimedPoint(413.0, new Point(309.89410400390625, 280.4914855957031)),
                new TimedPoint(414.0, new Point(564.6286010742188, 217.76089477539062)),
                new TimedPoint(415.0, new Point(287.36260986328125, 276.9635009765625)),
                new TimedPoint(416.0, new Point(560.09326171875, 226.0443115234375)),
                new TimedPoint(417.0, new Point(557.741455078125, 216.9669952392578)),
                new TimedPoint(418.0, new Point(547.0072021484375, 216.27186584472656)),
                new TimedPoint(420.0, new Point(549.7757568359375, 215.54684448242188)),
                new TimedPoint(421.0, new Point(543.8775634765625, 220.12818908691406)),
                new TimedPoint(422.0, new Point(539.9353637695312, 198.81858825683594)),
                new TimedPoint(423.0, new Point(538.634765625, 194.84942626953125)),
                new TimedPoint(424.0, new Point(525.7261962890625, 205.8837432861328)),
                new TimedPoint(425.0, new Point(528.0158081054688, 209.3490753173828)),
                new TimedPoint(426.0, new Point(516.1280517578125, 201.6374053955078)),
                new TimedPoint(428.0, new Point(507.71435546875, 211.9086456298828)),
                new TimedPoint(430.0, new Point(507.32952880859375, 198.89407348632812)),
                new TimedPoint(431.0, new Point(470.7920837402344, 203.55960083007812)),
                new TimedPoint(432.0, new Point(484.72808837890625, 193.50079345703125)),
                new TimedPoint(433.0, new Point(476.7325744628906, 188.44703674316406)),
                new TimedPoint(434.0, new Point(482.0361633300781, 184.67845153808594)),
                new TimedPoint(435.0, new Point(436.01141357421875, 321.5766296386719)),
                new TimedPoint(436.0, new Point(470.6485900878906, 190.6405792236328)),
                new TimedPoint(437.0, new Point(461.5006408691406, 191.01943969726562)),
                new TimedPoint(438.0, new Point(425.94775390625, 199.65834045410156)),
                new TimedPoint(439.0, new Point(445.8598327636719, 188.9493408203125)),
                new TimedPoint(440.0, new Point(411.6012268066406, 200.0704345703125)),
                new TimedPoint(441.0, new Point(429.38836669921875, 190.7281036376953)),
                new TimedPoint(442.0, new Point(419.9870910644531, 192.53384399414062)),
                new TimedPoint(443.0, new Point(404.9434814453125, 193.28514099121094)),
                new TimedPoint(444.0, new Point(401.0522766113281, 191.9495849609375)),
                new TimedPoint(445.0, new Point(385.9886474609375, 194.004150390625)),
                new TimedPoint(446.0, new Point(371.0223388671875, 201.6852264404297)),
                new TimedPoint(447.0, new Point(369.34832763671875, 194.94190979003906)),
                new TimedPoint(448.0, new Point(363.9318542480469, 195.62892150878906)),
                new TimedPoint(449.0, new Point(340.9764709472656, 199.89404296875)),
                new TimedPoint(450.0, new Point(338.25445556640625, 200.64593505859375)),
                new TimedPoint(451.0, new Point(318.08538818359375, 224.88243103027344)),
                new TimedPoint(452.0, new Point(290.94769287109375, 229.93740844726562)),
                new TimedPoint(453.0, new Point(294.9083251953125, 228.67247009277344)),
                new TimedPoint(454.0, new Point(275.91632080078125, 217.68861389160156)),
                new TimedPoint(455.0, new Point(255.06884765625, 266.0633544921875)),
                new TimedPoint(456.0, new Point(222.97975158691406, 231.82623291015625)),
                new TimedPoint(457.0, new Point(232.87335205078125, 249.2425537109375)),
                new TimedPoint(458.0, new Point(209.8844451904297, 252.89794921875)),
                new TimedPoint(459.0, new Point(197.291259765625, 249.7899169921875)),
                new TimedPoint(460.0, new Point(188.0839080810547, 253.89573669433594)),
                new TimedPoint(461.0, new Point(179.5569610595703, 275.51910400390625)),
                new TimedPoint(462.0, new Point(178.1430206298828, 284.14892578125)),
                new TimedPoint(463.0, new Point(161.63648986816406, 297.1679382324219)),
                new TimedPoint(464.0, new Point(153.85562133789062, 300.9812927246094)),
                new TimedPoint(465.0, new Point(134.9081573486328, 327.4660949707031)),
                new TimedPoint(466.0, new Point(122.98680114746094, 342.00396728515625)),
                new TimedPoint(467.0, new Point(131.77882385253906, 349.3769226074219)),
                new TimedPoint(468.0, new Point(124.6329116821289, 341.1186218261719)),
                new TimedPoint(469.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(470.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(471.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(472.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(473.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(474.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(475.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(476.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(477.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(478.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(479.0, new Point(117.7417984008789, 358.88824462890625)),
                new TimedPoint(480.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(481.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(482.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(483.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(484.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(485.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(486.0, new Point(160.56927490234375, 556.0787963867188)),
                new TimedPoint(487.0, new Point(307.8556213378906, 639.7841186523438)),
                new TimedPoint(488.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(489.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(490.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(491.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(492.0, new Point(308.8725891113281, 633.2608642578125)),
                new TimedPoint(493.0, new Point(443.2032775878906, 645.1068115234375)),
                new TimedPoint(494.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(495.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(496.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(497.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(498.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(499.0, new Point(468.24139404296875, 630.0108642578125)),
                new TimedPoint(500.0, new Point(602.7105102539062, 604.6414184570312))
        ));

        BezierTest test = new BezierTest(tiger);
        test.run(medianPointsDownswing);
//        test.showVid("fit.txt", 0.01);
    }

    public void run(ArrayList<TimedPoint> medianPointsDownswing) {
        Size size = new Size(790, 720);

        long initial = System.currentTimeMillis();
//        BezierFit fit = new BezierFit(testsPointsMedian, 4);
//        BezierFit fit = BezierFit.RANSAC(testData, 2, 3, 10, (int) 1e6);
//        BezierFit fit = BezierFit.RANSACRecursive(testsPointsMedian, 10, 11, 10, (int) 1e5); // backswing
        BezierFit fit = BezierFit.RANSACRecursive(data, 5, 20, 10, (int) 1e5); //downswing // 20 (tiger) or 10 (collin)

        System.out.println("Processing time: " + ((System.currentTimeMillis() - initial) / 1000.0));
        Point[] points = fit.getPoints(0.01);

        Mat mat = Mat.zeros(size, CvType.CV_8UC3);
//        fit.visualizeMinDist(mat);

        for(TimedPoint p : data) {
            Scalar color = new Scalar(255, 255, 255);

            double angle = Math.atan2(442 - p.y, p.x - 377);
            if(SwingAnalysis.inSection(angle, "top"))
                color = new Scalar(255, 0, 0);
            else if(SwingAnalysis.inSection(angle, "side-top"))
                color = new Scalar(0, 255, 100);
            else if(SwingAnalysis.inSection(angle, "side-bottom"))
                color = new Scalar(0, 100, 255);
            else if(SwingAnalysis.inSection(angle, "bottom"))
                color = new Scalar(100, 100, 255);

            Imgproc.circle(mat, p, 3, color, -1);
        }

        Mat tracer = Mat.zeros(size, CvType.CV_8UC3);
        for(int i = 0; i < points.length; i++) {
            Imgproc.circle(tracer, points[i], 3, new Scalar(255, 0, 0), -1);

            if(i > 0)
                Imgproc.line(tracer, points[i - 1], points[i], new Scalar(0, 255, 0));
        }

        Imgcodecs.imwrite("tracer.jpg", tracer);

        Core.add(mat, tracer, mat);

        for(Point p : fit.points) {
            Imgproc.circle(mat, p, 3, new Scalar(100, 255, 255), -1);
        }

        Imgproc.circle(mat, new Point(377, 442), 3, new Scalar(0, 0, 255));

//        Point[] timeSpacing = new Point[] {testData.get(0), testData.get(131), testData.get(243), testData.get(416), testData.get(561), testData.get(689), testData.get(852)};
//        for(Point p : timeSpacing)
//            Imgproc.circle(mat, p, 3, new Scalar(0, 255, 0), -1);

//        System.out.println("\n" + Arrays.toString(fit.getPoints(0.01)));

        HighGui.imshow("Test Bezier", mat);

        PrintWriter writer = null;

        try {
            writer = new PrintWriter(new FileWriter("fit.txt"));
        } catch(IOException e) {
            System.err.println("Unable to open file: fit.txt");
        }

        for(int i = 0; i < points.length; i++)
            writer.println(points[i].x + " " + points[i].y);
            
        writer.close();

//        showVid("fit.txt", 0.01);

//        mapFrameToT(fit, data);

        mapFrameToT(fit, medianPointsDownswing);

        HighGui.waitKey(0);
    }

    private void mapFrameToT(BezierFit fit, ArrayList<TimedPoint> medianDownswing) {
        ArrayList<Point> rawFrameToT = new ArrayList<Point>(); // x: frame, y: t

        ArrayList<TimedPoint> filtered = new ArrayList<TimedPoint>();

        if(medianDownswing.size() > 0)
            filtered.add(medianDownswing.get(0));

        for(int i = 1; i < medianDownswing.size(); i++) {
            if(!filtered.get(filtered.size() - 1).equals((Point) medianDownswing.get(i)) && fit.getMinDistance(medianDownswing.get(i), 1e6)[0] < 10)
                filtered.add(medianDownswing.get(i));
        }

        for(int i = 0; i < filtered.size(); i++) {
            TimedPoint p = filtered.get(i);

            double[] min = fit.getMinDistance(p, 0.001);
            double dist = min[0];
            double t = min[1];

            if (dist < 10) {
                rawFrameToT.add(new Point(p.frame, t));
                System.out.println("t: " + t);
            }
        }

        Mat medianDown = Mat.zeros(new Size(790, 720), CvType.CV_8UC3);
        for(Point p : medianDownswing)
            Imgproc.circle(medianDown, p, 3, new Scalar(255, 0, 0), -1);

        for(Point p : filtered)
            Imgproc.circle(medianDown, p, 3, new Scalar(0, 0, 255), -1);

        for(Point p : rawFrameToT)
            Imgproc.circle(medianDown, fit.getPoint(p.y), 3, new Scalar(0, 255, 0), -1);

        Mat medianDownGraph = Mat.zeros(new Size(200 * 10, 1000), CvType.CV_8UC3); // width: 124
        for(Point p : rawFrameToT)
            Imgproc.circle(medianDownGraph, new Point((p.x - 381) * 10, p.y * 1000 + 100), 3, new Scalar(0, 255, 0, -1));

        BezierFit speedFit = new BezierFit(rawFrameToT, 3);
        Point[] speedPoints = speedFit.getPoints(0.01);

        System.out.println("Speed points: " + Arrays.toString(speedPoints));

        for(int i = 1; i < speedPoints.length; i++) {
            Point p1 = speedPoints[i - 1];
            Point p2 = speedPoints[i];

            Imgproc.line(medianDownGraph, new Point((p1.x - 381) * 10, p1.y * 1000 + 100), new Point((p2.x - 381) * 10, p2.y * 1000 + 100), new Scalar(100, 100, 200), 2);
        }

//        Imgproc.resize(medianDownGraph, medianDownGraph, new Size(medianDownGraph.width() * 10, medianDownGraph.height() * 10));

        HighGui.imshow("Median downswing", medianDown);
        HighGui.imshow("Median Downswing Graph", medianDownGraph);

        HighGui.waitKey(0);
    }

    private void showVid(String file, double dt) {
        VideoCapture vid = new VideoCapture("src/res/tigerdriver.mp4");
        int startFrame = 381;
        int endFrame = 500;

        int frameNum = startFrame;
        vid.set(Videoio.CAP_PROP_POS_FRAMES, frameNum);

        Mat out = Mat.zeros(new Size(790, 720), CvType.CV_8UC3);

        Scanner scan = null;

        try {
            scan = new Scanner(new FileReader(file));
        } catch(FileNotFoundException e) {
            System.err.println("File not found: " + file);
        }

        Point[] points = new Point[(int) ((1 / dt) + 0.5)];
        int idx = 0;
        while(scan.hasNext()) {
            points[idx] = new Point(scan.nextDouble(), scan.nextDouble());

            idx++;
        }

        Mat img = new Mat();

        int pointIdx = 0;
        double pointsPerFrame = points.length / (double) (endFrame - startFrame);
        double pointBufferLen = 0;

        System.out.println("ppf: " + pointsPerFrame);

        boolean ok = vid.read(img);
        while(frameNum < endFrame) {
            pointBufferLen += pointsPerFrame;

            int end = (int) (pointIdx + pointBufferLen);
            for(int i = pointIdx; i < end; i++) {
                Imgproc.circle(out, points[i], 3, new Scalar(255, 255, 255), -1);

                pointIdx++;
            }

            pointBufferLen = pointBufferLen - (int) pointBufferLen;

            Mat animation = new Mat();
            Core.add(img, out, animation);
            HighGui.imshow("Animation", animation);

            ok = vid.read(img);

            frameNum++;

            HighGui.waitKey((int) (1000 / (vid.get(Videoio.CAP_PROP_FPS) * 0.5)));
        }

//        for(int i = 0; i < points.length; i++) {
//            Imgproc.circle(out, points[i], 3, new Scalar(100, 200, 80), -1);
//
//            HighGui.imshow("Animation", out);
//
//            HighGui.waitKey((int) ((1.0 / 30) * 1000));
//        }

        HighGui.imshow("Animation", out);

        HighGui.waitKey(0);
    }
}
