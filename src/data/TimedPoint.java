package data;

import org.opencv.core.Point;

import java.io.Serializable;

public class TimedPoint extends Point implements Serializable {
    public final double frame;

    public TimedPoint(double frame, Point pos) {
        super(pos.x, pos.y);

        this.frame = frame;
    }

    @Override
    public String toString() {
        return String.format("Frame: %f | (%f, %f)", frame, x, y);
    }
}
