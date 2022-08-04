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

    public SwingFit(BezierFit pathFit, BezierFit speedFit) {
        this.pathFit = pathFit;
        this.speedFit = speedFit;
    }

    /**
     * Shows a video of the fit on top of the swing video
     * @param filePath The file path to the video
     * @param startFrame The frame to start the video
     * @param endFrame The frame to end the video
     */
    public void showVid(String filePath, int startFrame, int endFrame) {
        VideoCapture vid = new VideoCapture(filePath);
        vid.set(Videoio.CAP_PROP_POS_FRAMES, startFrame);
        int fps = (int) vid.get(Videoio.CAP_PROP_FPS);

        int frameNum = startFrame;

        Mat vidCap = new Mat();
        Mat tracer = null;

        boolean ok = vid.read(vidCap);

        tracer = Mat.zeros(vidCap.size(), CvType.CV_8UC3);

        LUT lut = speedFit.createLUT(0.01);
        int[] pascal = pathFit.getGlobalPascal();

        Point prev = null;

        while(ok) {
            Mat animation = Mat.zeros(vidCap.size(), CvType.CV_8UC3);

            if(frameNum >= lut.getLowerBound() && frameNum <= lut.getUpperBound()) {
                double t = lut.get(frameNum);
                Point curr = pathFit.getPoint(pascal, t);

                if(prev != null && t >= 0 && t <= 1)
                    Imgproc.line(tracer, prev, curr, new Scalar(100, 255, 100));

                Core.addWeighted(vidCap, 0.4, tracer, 0.6, 0, animation);

                prev = curr;
            }

            HighGui.imshow("Swing Tracer Animation", animation);
            HighGui.waitKey(1000 / fps);

            ok = vid.read(vidCap);
            frameNum++;

            if(frameNum > endFrame)
                ok = false;
        }
    }

    public BezierFit getPathFit() {
        return pathFit;
    }

    public BezierFit getSpeedFit() {
        return speedFit;
    }
}
