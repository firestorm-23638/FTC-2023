package org.firstinspires.ftc.teamcode;

public class Vector {
    public double x = 0;
    public double y = 0;

    public double toDegrees(double rad) {
        return (rad / Math.PI) * 180;
    }

    public double toRadians(double deg) {
        return deg * (Math.PI / 180);
    }

    public double getAngle() {
        return toDegrees(Math.atan2(y, x));

    }

    public double getMag() {
        return Math.sqrt((x*x) + (y*y));
    }

    public void setAngle(double newAng) {
        x = Math.cos(toRadians(newAng)) * getMag();
        y = Math.sin(toRadians(newAng)) * getMag();
    }
};
