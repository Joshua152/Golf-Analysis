import data.Circle;
import data.SwingFit;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class Main {
    public static void main(String[] args) {
        Main main = new Main();
        main.run();
    }

    public void run() {
        // TODO: DYNAMIC END FRAME

        long start = System.currentTimeMillis();

        String filePath = "src/res/colliniron.mp4";

        KeypointTracking tracking = new KeypointTracking(filePath, 0, 660, KeypointTracking.FLAG_DISPLAY_INFO); // endFrame: 515 (tiger), 653 (collin)
        System.out.println("Fitting...");
        SwingFit fit = tracking.fit();
        System.out.println("Fitting... Complete");

//        Analysis analysis = new Analysis(fit, tracking.getBall(), tracking.getFPS());
        Analysis analysis = new Analysis(fit, new Circle(new Point(431, 646), 6), 29);

        System.out.println("Processing Time: " + ((System.currentTimeMillis() - start) / 1000.0));

        for(int i = 0; i < 5; i++) {
            AtomicReference<Double> impactSpeed = new AtomicReference<>((double) -1);
            AtomicReference<Double> impactAttackAngle = new AtomicReference<>((double) -1);

            int slomoMultiplier = 26;

            fit.showVid(filePath, tracking.getDownswingFrame(), Integer.MAX_VALUE, (out, frame) -> {
                double speed = analysis.getSpeed(frame);
                double attackAngle = analysis.getAttackAngle(frame);

                Imgproc.putText(out, "Speed: " + speed, new Point(0, 25), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
                Imgproc.putText(out, "Adjusted: " + (speed * slomoMultiplier), new Point(0, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
                Imgproc.putText(out, "AA: " + attackAngle, new Point(0, 75), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));

                if(frame >= analysis.getImpactFrame()) {
                    if(impactSpeed.get() == -1) {
                        impactSpeed.set(speed * slomoMultiplier);
                        impactAttackAngle.set(attackAngle);
                    }

                    Imgproc.putText(out, "Impact", new Point(0, out.rows() - 75), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
                    Imgproc.putText(out, "Speed: " + impactSpeed.get(), new Point(0, out.rows() - 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
                    Imgproc.putText(out, "AA: " + impactAttackAngle.get(), new Point(0, out.rows() - 25), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
                }
            });
        }

        HighGui.waitKey(0);
    }
}