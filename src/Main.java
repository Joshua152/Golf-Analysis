import data.SwingFit;
import org.opencv.highgui.HighGui;

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

        String filePath = "src/res/tigerdriver.mp4";

        KeypointTracking tracking = new KeypointTracking(filePath, 0, 515, KeypointTracking.FLAG_DISPLAY_INFO); // endFrame: 515 (tiger), 585 (collin)
        System.out.println("Fitting...");
        SwingFit fit = tracking.fit();
        System.out.println("Fitting... Complete");

        System.out.println("Processing Time: " + ((System.currentTimeMillis() - start) / 1000.0));

        for(int i = 0; i < 15; i++)
            fit.showVid(filePath, tracking.getDownswingFrame(), Integer.MAX_VALUE);

        HighGui.waitKey(0);
    }
}
