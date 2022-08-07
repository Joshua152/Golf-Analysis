import data.Circle;
import data.SwingFit;
import org.opencv.core.Point;

public class Analysis {
    private final double BALL_DIAMETER = 1.68; // inches

    private SwingFit swing;
    private Circle ball;

    private double impactFrame;

    public Analysis(SwingFit swing, Circle ball) {
        this.swing = swing;
        this.ball = ball;

        impactFrame = 0;

        setup();
    }

    private void setup() {
        // won't find the exact frame, but good enough
        double[] minDist = swing.getPathFit().getMinDistance(ball.getCenter(), 0.01);
        double minT = minDist[1];

        impactFrame = swing.getPathFrame(minT);

        System.out.println("Impact Frame: " + impactFrame);
    }

    /**
     * Gets the swing speed at any given frame
     * @param frame The frame to get the swing speed of (must be within the bounds of SwingFit)
     * @return Returns the speed of the golf swing in mph
     */
    public double getSpeed(double frame) {
        return 0;
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    private double inches(double pixels) {
        return pixels * (ball.getRadius() / (BALL_DIAMETER / 2));
    }
}
