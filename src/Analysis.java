import data.Circle;
import data.SwingFit;
import org.opencv.core.Point;

public class Analysis {
    private final double BALL_DIAMETER = 1.68; // inches

    private SwingFit swing;
    private Circle ball;

    private int videoFPS;

    private double impactFrame;

    public Analysis(SwingFit swing, Circle ball, int videoFPS) {
        this.swing = swing;
        this.ball = ball;

        this.videoFPS = videoFPS;

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

    public double getAttackAngle(double frame) {
        if(frame < swing.getLUT().getLowerBound() || frame > swing.getLUT().getUpperBound())
            return -1;

        double t1 = swing.getPathT(frame);
        double t2 = t1 + 0.01;

        Point p1 = swing.getPathFit().getPoint(t1);
        Point p2 = swing.getPathFit().getPoint(t2);

        Point normalized = new Point(p2.x - p1.x, p1.y - p2.y);
        return Math.atan2(normalized.y, normalized.x) * 180 / Math.PI;
    }

    /**
     * Gets the swing speed at any given frame
     * @param frame The frame to get the swing speed of (must be within the bounds of SwingFit)
     * @return Returns the speed of the golf swing in mph or -1 if the frame # is out of bounds of the path
     */
    public double getSpeed(double frame) {
        if(frame < swing.getLUT().getLowerBound() || frame > swing.getLUT().getUpperBound())
            return -1;

        // change if frame is max

        double t1 = swing.getPathT(frame);
        double t2 = t1 + 0.01;

        double frame2 = swing.getPathFrame(t2);
        double dt = (frame2 - frame) / videoFPS; // seconds

        Point p1 = swing.getPathFit().getPoint(t1);
        Point p2 = swing.getPathFit().getPoint(t2);

        double ds = Math.abs(inches(distance(p1, p2))); // change in distance

        double speed = ds / dt; // inches per second

        return speed / 63360 * 60 * 60; // convert to mph
    }

    private double distance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    private double inches(double pixels) {
        return pixels * (BALL_DIAMETER / 2) / ball.getRadius();
    }

    public double getImpactFrame() {
        return impactFrame;
    }
}
