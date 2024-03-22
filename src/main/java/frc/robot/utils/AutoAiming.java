package frc.robot.utils;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoAiming {
    public final boolean usingLerp;
    public static final int degree = 3;
    public static final boolean isCollecting = true;
    public static final ConfigurableParameter<Double> bucketSize = new ConfigurableParameter<>(0.1, "Bucket Size");

    private final ConfigurableParameter<Double> autoAimOffset = new ConfigurableParameter<Double>(0.0,
            "Auto Aim Temporary Offset");
    private boolean runningWithoutData = false;
    private PolynomialRegression polynomialRegression;
    private JSONArray data;
    private double[] distances = {
            1.25,
            1.755,
            2.324,
            2.8,
            3.993
    };
    private double[] angles = {
            0.122, // ORIGINALLY 0.119, CHANGE BACK IF BORKEN!
            0.0965,
            0.0849,
            0.0675,
            0.0469,
    };
    // THIS IS THE BAD (NEW) DATA
    // private double[] distances = {
    // 1.248,
    // 1.590,
    // 1.860,
    // 2.313,
    // 2.980,
    // 3.714
    // };
    // private double[] angles = {
    // 0.114,
    // 0.104,
    // 0.089,
    // 0.072,
    // 0.0615,
    // 0.041
    // };
    private LinearInterpolation linearInterpolation;

    public AutoAiming() throws FileNotFoundException {
        usingLerp = false;
        FileInputStream fis = new FileInputStream(new File(getPath()));
        JSONTokener tokener = new JSONTokener(fis);
        data = new JSONArray(tokener);
        // TODO: Remove after testing
        if (usingLerp) {
            linearInterpolation = new LinearInterpolation(distances, angles);
        }
        // updateDataFromJSON();
    }

    public AutoAiming(boolean lerp) throws FileNotFoundException {
        usingLerp = lerp;
        FileInputStream fis = new FileInputStream(new File(getPath()));
        JSONTokener tokener = new JSONTokener(fis);
        data = new JSONArray(tokener);
        if (usingLerp) {
            linearInterpolation = new LinearInterpolation(distances, angles);
        }
        // updateDataFromJSON();
    }

    public double getAngle(double distance) {
        if (usingLerp) {
            return lerp(distance) + autoAimOffset.get();
        }
        if (runningWithoutData) {
            DriverStation.reportWarning("Auto aiming running with insufficient data, generated angles invalid", false);
            return 0;
        }
        return polynomialRegression.predict(distance);
    }

    /**
     * Saves the data and updates the polynomial regression
     * 
     * @param distance
     * @param angle
     */
    public void addData(double distance, double angle, double shooterTopVelocity, double shooterBottomVelocity,
            double drivebaseVelocityX,
            double drivebaseVelocityY) {
        if (!isCollecting) {
            return;
        }
        JSONObject point = new JSONObject();
        point.put("distance", distance);
        point.put("angle", angle);
        point.put("shooterTopVelocity", shooterTopVelocity);
        point.put("shooterBottomVelocity", shooterBottomVelocity);
        point.put("drivebaseVelocityX", drivebaseVelocityX);
        point.put("drivebaseVelocityY", drivebaseVelocityY);
        point.put("made", false);
        point.put("reviewed", false);
        data.put(point);
        try {
            File file = new File(getPath());
            file.createNewFile();
            Files.write(file.toPath(), data.toString(2).getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @SuppressWarnings("unused")
    private void updateDataFromJSON() {
        ArrayList<Double> distancesTemp = new ArrayList<>();
        ArrayList<Double> anglesTemp = new ArrayList<>();
        for (int i = 0; i < data.length(); i++) {
            JSONObject point = data.getJSONObject(i);
            if (!point.getBoolean("reviewed") || !point.getBoolean("made"))
                continue;
            distancesTemp.add(point.getDouble("distance"));
            anglesTemp.add(point.getDouble("angle"));
        }
        double longestDistance = 0;
        distances = new double[distancesTemp.size()];
        angles = new double[anglesTemp.size()];
        for (int i = 0; i < distancesTemp.size(); i++) {
            distances[i] = distancesTemp.get(i);
            if (distances[i] > longestDistance) {
                longestDistance = distances[i];
            }
            angles[i] = anglesTemp.get(i);
        }

        if (usingLerp) {
            linearInterpolation = new LinearInterpolation(distances, angles);
        }

        int bucketCount = (int) Math.ceil(longestDistance / bucketSize.get()) + 1;
        ArrayList<ArrayList<Double>> distanceBuckets = new ArrayList<>();
        ArrayList<ArrayList<Double>> angleBuckets = new ArrayList<>();
        for (int i = 0; i < bucketCount; i++) {
            distanceBuckets.add(new ArrayList<>());
            angleBuckets.add(new ArrayList<>());
        }
        for (int i = 0; i < distances.length; i++) {
            int bucket = (int) Math.floor(distances[i] / bucketSize.get());
            distanceBuckets.get(bucket).add(distances[i]);
            angleBuckets.get(bucket).add(angles[i]);
        }

        ArrayList<Double> medianDistances = new ArrayList<>();
        ArrayList<Double> medianAngles = new ArrayList<>();
        // Add the median of each bucket to the arrays
        for (int i = 0; i < bucketCount; i++) {
            if (distanceBuckets.get(i).size() == 0) {
                continue;
            }
            medianDistances.add(median(distanceBuckets.get(i)));
            medianAngles.add(median(angleBuckets.get(i)));
        }

        double[] processedDistances = new double[medianDistances.size()];
        double[] processedAngles = new double[medianAngles.size()];
        for (int i = 0; i < medianDistances.size(); i++) {
            processedDistances[i] = medianDistances.get(i);
            processedAngles[i] = medianAngles.get(i);
        }
        try {
            polynomialRegression = new PolynomialRegression(processedDistances, processedAngles, degree);
            runningWithoutData = false;
        } catch (IllegalArgumentException e) {
            DriverStation.reportWarning("Not enough data to create a polynomial regression", false);
            runningWithoutData = true;
        }
    }

    /**
     * Get the path of the data file
     * 
     * @return the path of the data file
     */
    private String getPath() {
        if (RobotBase.isReal()) {
            return isCollecting ? USBLocator.getUSBPath() + "/auto-aiming.json"
                    : Filesystem.getDeployDirectory() + "/auto-aiming.json";
        } else {
            return Filesystem.getOperatingDirectory() + "/auto-aiming-sim.json";
        }
    }

    private double median(ArrayList<Double> list) {
        Collections.sort(list);
        int length = list.size();
        if (length % 2 == 0) {
            return (list.get(length / 2) + list.get(length / 2 - 1)) / 2;
        } else {
            return list.get(length / 2);
        }
    }

    private double lerp(double distance) {
        if (distances.length < 2) {
            DriverStation.reportWarning(
                    "Auto aiming running with insufficient data, generated angles invalid (lerp mode)", false);
            return 0;
        }
        return linearInterpolation.interpolate(distance);
    }
}
