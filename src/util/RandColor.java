package util;

import org.opencv.core.Scalar;

public class RandColor {
    private static Scalar[] colors;

    static {
        colors = new Scalar[1000];

//        for(int i = 0; i < colors.length; i++)
//            colors[i] = new Scalar((int) (Math.random() * 256), (int) (Math.random() * 256), (int) (Math.random() * 256));

        for(int i = 0; i < colors.length / 2; i++)
            colors[i] = new Scalar(i * (250 / (colors.length / 2.0)), ((colors.length / 2.0) - i) * (250 / (colors.length / 2.0)), 0);

        for(int i = colors.length / 2; i < colors.length; i++)
            colors[i] = new Scalar(((colors.length / 2.0) - i) * (250 / (colors.length / 2.0)), 0, i * (250 / (colors.length / 2.0)));
    }

    public static Scalar get(int i) {
        return colors[i % colors.length];
    }
}
