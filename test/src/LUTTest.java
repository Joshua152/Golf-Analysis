import org.opencv.core.Point;
import util.LUT;

public class LUTTest {
    public static void main(String[] args) {
        LUTTest test = new LUTTest();
        test.run();
    }

    private void run() {
        LUT lut = new LUT(new Point[] {
                new Point(0, 1),
                new Point(1, 3),
                new Point(2, 6),
                new Point(3, 7),
                new Point(6, 10)
        });

        System.out.println("Test 1: " + (lut.get(0) == 1));
        System.out.println("Test 2: " + (lut.get(0.5) == 2));
        System.out.println("Test 3: " + (lut.get(5) == 9));
        System.out.println("Test 4: " + (lut.get(6) == 10));
    }
}
