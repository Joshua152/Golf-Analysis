import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class SpeedTest {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args) {
        SpeedTest test = new SpeedTest();
        test.run();
    }

    private void run() {
        VideoCapture cap = new VideoCapture("src/res/tigerdriver.mp4");
        int fps = (int) cap.get(Videoio.CAP_PROP_FPS) * 26;

        int frameSkip = 20;

        Mat[] frameBuffer = new Mat[((int) cap.get(Videoio.CAP_PROP_FRAME_COUNT) / frameSkip) + 1];
        for(int i = 0; i < (int) cap.get(Videoio.CAP_PROP_FRAME_COUNT); i++) {
            Mat read = new Mat();
            cap.read(read);

            if (i % frameSkip == 0)
                frameBuffer[i / frameSkip] = read;
        }

        for(int i = 0; i < frameBuffer.length; i++) {
            HighGui.imshow("Vid", frameBuffer[i]);
            HighGui.waitKey(1000 / (fps / frameSkip));
        }
    }
}
