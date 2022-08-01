import org.opencv.highgui.HighGui;
import util.BezierFit;

public class Main {
    public static void main(String[] args) {
        Main main = new Main();
        main.run();
    }

    public void run() {
        // TODO: BALL IMPACT FRAME
        // TODO: MORE ROBUST DOWNSWING FRAME (USE X INSTEAD)
        // TODO: DYNAMIC END FRAME

        long start = System.currentTimeMillis();

        KeypointTracking tracking = new KeypointTracking("src/res/tigerdriver.mp4", 0, 515, KeypointTracking.FLAG_DISPLAY_INFO);
        System.out.println("Fitting...");
        BezierFit fit = tracking.fit();
        System.out.println("Fitting... Complete");

        System.out.println("Processing Time: " + ((System.currentTimeMillis() - start) / 1000.0));

        HighGui.waitKey(0);
    }
}
