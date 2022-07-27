package data;

import org.opencv.core.Point;

public class TimedPoint extends Point {
    public final double frame;

    public TimedPoint(double frame, Point pos) {
        super(pos.x, pos.y);

        this.frame = frame;
    }
}
