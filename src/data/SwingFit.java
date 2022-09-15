package data;

import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import util.BezierFit;
import util.LUT;

public class SwingFit {
    private BezierFit pathFit;
    private BezierFit speedFit;

    private LUT lut;

    public SwingFit(BezierFit pathFit, BezierFit speedFit) {
        this.pathFit = pathFit;
        this.speedFit = speedFit;

        lut = speedFit.createLUT(0.01);
    }

    /**
     * Shows a video of the fit on top of the swing video
     * @param filePath The file path to the video
     * @param startFrame The frame to start the video
     * @param endFrame The frame to end the video
     */
    public void showVid(String filePath, int startFrame, int endFrame, ShowFrameCallback callback) {
        VideoCapture vid = new VideoCapture(filePath);
        vid.set(Videoio.CAP_PROP_POS_FRAMES, startFrame);
        int fps = (int) vid.get(Videoio.CAP_PROP_FPS);

        int frameNum = startFrame;

        Mat vidCap = new Mat();
        Mat tracer = null;

        boolean ok = vid.read(vidCap);

        tracer = Mat.zeros(vidCap.size(), CvType.CV_8UC3);

        int[] pascal = pathFit.getGlobalPascal();

        Point prev = null;

        while(ok) {
            Mat animation = vidCap.clone();

            if(frameNum >= lut.getLowerBound() && frameNum <= lut.getUpperBound()) {
                double t = lut.getY(frameNum);
                Point curr = pathFit.getPoint(pascal, t);

                if(prev != null && t >= 0 && t <= 1)
                    Imgproc.line(tracer, prev, curr, new Scalar(100, 255, 100));

                Core.addWeighted(animation, 0.4, tracer, 0.6, 0, animation);

                prev = curr;
            }

            callback.onShowFrame(animation, frameNum);

            HighGui.imshow("Swing Tracer Animation", animation);
            HighGui.waitKey(1000 / fps);

            ok = vid.read(vidCap);
            frameNum++;

            if(frameNum > endFrame)
                ok = false;
        }
    }

    /**
     * Gets the pathFit t value from the given frame
     * @param frame The frame to get the corresponding t value of
     * @return The t value [0, 1] of the pathFit from the given frame
     */
    public double getPathT(double frame) {
        return lut.getY(frame);
    }

    /**
     * Gets the pathFit frame number from the given t
     * @param t The t value to get teh corresponding frame number of
     * @return The frame number from the given t value [0, 1]
     */
    public double getPathFrame(double t) {
        return lut.getX(t);
    }

    public BezierFit getPathFit() {
        return pathFit;
    }

    public BezierFit getSpeedFit() {
        return speedFit;
    }

    /**
     * x: frame, y: t
     * @return Returns the LUT from speedFit
     */
    public LUT getLUT() {
        return lut;
    }

    /**
     * Callback for showing the video
     */
    public interface ShowFrameCallback {
        /**
         * Method that gets called before each frame is shown
         * @param out The mat frame to be shown
         * @param frame The frame that will get displayed
         */
        void onShowFrame(Mat out, int frame);
    }
}
