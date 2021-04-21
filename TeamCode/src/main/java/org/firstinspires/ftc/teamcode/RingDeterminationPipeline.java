package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;
import java.util.Map;

public class RingDeterminationPipeline extends OpenCvPipeline {

    public enum RingPosition {
        FOUR,
        ONE,
        BLANK,
        NONE
    }


    public static double[][] getThresholdData() {
        return thresholdData;
    }

    public static void setThresholdData(double[][] thresholdData) {
        RingDeterminationPipeline.thresholdData = thresholdData;
    }

    public double getTotalThreshold() {
        return totalThreshold;
    }

    public void setTotalThreshold(double totalThreshold) {
        this.totalThreshold = totalThreshold;
    }

    public static double[][] getThresholdPercentageData() {
        return thresholdPercentageData;
    }

    public static void setThresholdPercentageData(double[][] thresholdPercentageData) {
        RingDeterminationPipeline.thresholdPercentageData = thresholdPercentageData;
    }

    public static double[][] thresholdData = new double[3][3];
    public double totalThreshold;
    public static double[][] thresholdPercentageData = new double[3][3];


    int specifiedRegion_x = 1;

    public int getSpecifiedRegion_x() {
        return specifiedRegion_x;
    }

    public void setSpecifiedRegion_x(int specifiedRegion_x) {
        this.specifiedRegion_x = specifiedRegion_x;
    }

    public int getSpecifiedRegion_y() {
        return specifiedRegion_y;
    }

    public void setSpecifiedRegion_y(int specifiedRegion_y) {
        this.specifiedRegion_y = specifiedRegion_y;
    }

    int specifiedRegion_y = 1;

    /*
     * An enum to define the skystone position
     */

    public double getSingleBoxInfluenceConstant() {
        return singleBoxInfluenceConstant;
    }

    public final double singleBoxInfluenceConstant = 0.12;

/*
    static StageAnalyze processImageandMovetoNextStage = (stage, mat, pipeline) -> {
        pipeline.getThresholdDataRegion(mat, stage.offsetX, stage.offsetY);

        return stage.nextNode;
    };
*/


    void getThreshold(Mat mat, int offsetX, int offsetY) {
        getThresholdDataRegion(mat, offsetX, offsetY);
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(
            181,
            98);

    static final int REGION_WIDTH = 35;
    static final int REGION_HEIGHT = 25;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 135;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        Point region2_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region2_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    double avg1;

    public void setPosition(RingPosition position) {
        this.position = position;
    }

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile RingPosition position = RingPosition.BLANK;

    RingPosition getPosition() {
        return position;
    }

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    public double[][] getThresholdData(Mat input) {
        inputToCb(input);
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                Mat subRegion = Cb.submat(
                        new Rect
                                (new Point(
                                        region1_pointA.x + i * REGION_WIDTH,
                                        region1_pointA.y + j * REGION_HEIGHT),
                                        new Point(
                                                region1_pointB.x + i * REGION_WIDTH,
                                                region1_pointB.y + j * REGION_HEIGHT)));
                thresholdData[i + 1][j + 1] = Core.mean(subRegion).val[0];
                totalThreshold += Core.mean(subRegion).val[0];
            }
        }
        return thresholdData;
    }

    public void getThresholdDataRegion(Mat input, int offsetX, int offsetY) {
        inputToCb(input);
        Mat subRegion = Cb.submat(
                new Rect
                        (new Point(
                                region1_pointA.x + offsetX * REGION_WIDTH,
                                region1_pointA.y + offsetY * REGION_HEIGHT),
                                new Point(
                                        region1_pointB.x + offsetX * REGION_WIDTH,
                                        region1_pointB.
                                                y + offsetY * REGION_HEIGHT)));
        thresholdData[offsetX + 1][-offsetY + 1] = Core.mean(subRegion).val[0];
        totalThreshold += Core.mean(subRegion).val[0];
    }

    public double[][] getThresholdPercentageData(double[][] thresholdData) {
        for (int i = 0; i <= 2; i++) {
            for (int j = 0; j <= 2; j++) {
                thresholdPercentageData[i][j] = thresholdData[i][j] / totalThreshold;
            }
        }
        return thresholdPercentageData;
    }

    public void moveRegionBiggest(double[][] thresholdPercentageData) {
        double maxValue = 0;
        int[] maxCombination = {0, 0};
        for (int i = 0; i <= 2; i++) {
            for (int j = 0; j <= 2; j++) {
                if (thresholdPercentageData[i][j] > maxValue) {
                    maxValue = thresholdPercentageData[i][j];
                    maxCombination = new int[]{i, j};

                }
            }
        }
        region1_pointA = new Point(region1_pointA.x + (maxCombination[0] - 1) * REGION_WIDTH, region1_pointA.y + (maxCombination[1] - 1) * REGION_HEIGHT);
        region1_pointB = new Point(region1_pointB.x + (maxCombination[0] - 1) * REGION_WIDTH, region1_pointB.y + (maxCombination[1] - 1) * REGION_HEIGHT);
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        setSpecifiedRegion_x(maxCombination[0]);
        setSpecifiedRegion_y(maxCombination[1]);
    }

    public void moveRegionSlightlyThreeWay() {
    }


    public void moveRegionSlightlyCardinal(double[][] thresholdPercentageData) {
        for (int i = 0; i < thresholdPercentageData.length; i++) {
            for (int j = 0; j < thresholdPercentageData.length; i++) {
//                if()
            }
        }
    }

    public int getFOUR_RING_THRESHOLD() {
        return FOUR_RING_THRESHOLD;
    }

    public int getONE_RING_THRESHOLD() {
        return ONE_RING_THRESHOLD;
    }

    @Override
    public Mat processFrame(Mat input) {
        long startTime = System.currentTimeMillis();
        //currentStage =RingVisualizationStage.Center;
        getThresholdPercentageData(getThresholdData(input));
//        while (currentStage != RingVisualizationStage.Done) {
//            if (!currentStage.isAnalyzing()) {
//                currentStage.processAction(input, this);
//            } else {
//                currentStage.analyzeAction(input, this);
//            }
//        }

        avg1=Core.mean(region1_Cb).val[0];
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
// Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            position = RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            position = RingPosition.ONE;
        }else{
            position = RingPosition.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        double elapsedTime = (System.currentTimeMillis() - startTime) / 1000;
        return input;
    }

    public double[][] getThresholdDataAnalysis() {
        return thresholdData;
    }

    public double[][] getThresholdPercentageDataAnalysis() {
        return thresholdPercentageData;
    }
    public RingPosition getRingPosition() {
        return position;
    }
    public double getAnalysis(){
        return avg1;
    }
}

