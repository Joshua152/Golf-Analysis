import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class ProcessingTest {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args) {
        ProcessingTest test = new ProcessingTest();
        test.run();
    }

    private void run() {
        String fileName = "src/res/tigerdriver.mp4";
        VideoCapture cap = new VideoCapture(fileName);
        int fps = (int) cap.get(Videoio.CAP_PROP_FPS);

//        Mat capFrame = Imgcodecs.imread("src/res/circle test.png");

        Mat capFrame = new Mat();
        boolean ok = cap.read(capFrame);
        while(ok) {
            Mat gray = new Mat();
            Imgproc.cvtColor(capFrame, gray, Imgproc.COLOR_BGR2GRAY);

            Imgproc.medianBlur(gray, gray, 5);

            HighGui.imshow("Gray", gray);

            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 200, 300);

            HighGui.imshow("Edges", edges);

            Mat circles = new Mat();
            Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, (double) gray.rows() / 16, 300, 15,
                    1, 15);

            System.out.printf("Circle #: %d\n", circles.cols());

            System.out.println(circles.rows() + " " + circles.cols());

            for(int i = 0; i < circles.cols(); i++) {
                double[] c = circles.get(0, i);

                Point center = new Point(Math.round(c[0]), Math.round(c[1]));
                int radius = (int) Math.round(c[2]);

                Imgproc.circle(capFrame, center, 3, new Scalar(0, 100, 100), -1);
                Imgproc.circle(capFrame, center, radius, new Scalar(255, 0, 255), 2);
            }

            System.out.println("Showing circles...");
            HighGui.imshow("Hough Circles", capFrame);
            HighGui.waitKey(1000 / fps);

            ok = cap.read(capFrame);
        }
    }
}
