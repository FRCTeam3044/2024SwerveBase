package frc.robot.utils;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public class AutoAiming {
    public static final int degree = 3;
    public static final boolean isCollecting = true;

    private boolean runningWithoutData = false;
    private PolynomialRegression polynomialRegression;
    private JSONArray data;
    private double[] distances;
    private double[] angles;

    // /U/toiwehiotwegoweiguowehoigwihobwuogf.json

    public AutoAiming() throws FileNotFoundException {
        FileInputStream fis = new FileInputStream(new File(getPath()));
        JSONTokener tokener = new JSONTokener(fis);
        data = new JSONArray(tokener);
        updateDataFromJSON();
    }

    public double getAngle(double distance) {
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
    public void addData(double distance, double angle, double shooterVelocity, double drivebaseVelocity) {
        if (!isCollecting) {
            return;
        }
        JSONObject point = new JSONObject();
        point.put("distance", distance);
        point.put("angle", angle);
        point.put("shooterVelocity", shooterVelocity);
        point.put("drivebaseVelocity", drivebaseVelocity);
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
        distances = new double[distancesTemp.size()];
        angles = new double[anglesTemp.size()];
        for (int i = 0; i < distancesTemp.size(); i++) {
            distances[i] = distancesTemp.get(i);
            angles[i] = anglesTemp.get(i);
        }

        try {
            polynomialRegression = new PolynomialRegression(distances, angles, degree);
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
            return isCollecting ? "/U/auto-aiming.json" : Filesystem.getDeployDirectory() + "/auto-aiming.json";
        } else {
            return Filesystem.getOperatingDirectory() + "/auto-aiming-sim.json";
        }
    }
}
