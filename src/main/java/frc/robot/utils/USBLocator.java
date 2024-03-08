package frc.robot.utils;

import java.io.File;

public class USBLocator {
    private static String usbPath;

    public static String getUSBPath() {
        // if (usbPath == null) {
        // if (new File("/U/logs").exists()) {
        // usbPath = "/U";
        // } else {
        // usbPath = "/V";
        // }
        // }
        // System.out.println(usbPath);
        // return usbPath;
        return "/U";
    }
}
