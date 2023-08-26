package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.UnitsTools;

public final class Constants {

    public static final class ElevatorConstants {

        //distances in encoder counts

        public static final int POSITION_TOLERANCE = 1;

        public static final double UPPER_POSITION_LIMIT = 35.00;

        public static final int LOWER_POSITION_LIMIT = 1;

        public static final double JOG_UP_POWER = +.5;

        public static final double JOG_DOWN_POWER = -.3;

        public static final double LOW_JUNCTION = 13.5;
        public static final double MID_JUNCTION = 23.5;
        public static final double HIGH_JUNCTION = 33.5;

        public static final double GROWND_JUNCTION = 0;
        public static final double ENCODER_COUNTS_PER_INCH = 84.75;
        public static final double POSITION_Kp = 1;

        public static final double POSITION_Kd = .00008;
        public static final double POSITION_Kg = .0002;


        public static final double POSITION_UP_POWER = .5;
        public static final double POSITION_DOWN_POWER = -.3;


    }

    public static final class DriveConstants {

        public static final double MOTOR_GEAR_RATIO = 13.7;

        public static final double GEARBOX_RATIO = 1;
        public static final double WHEEL_DIAMETER_INCH = 4;

        public static final double WHEELWIDTH = 13 * .0254;//16.25

        public static final double TRACKWIDTH = 16.25 * .0254;//13
        public static final double WHEEL_CIRCUMFERENCE_INCH = Math.PI * WHEEL_DIAMETER_INCH;
        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 537.7;
        public static final double ENCODER_COUNTS_PER_INCH = .0254 / ((ENCODER_COUNTS_PER_MOTOR_REV) / WHEEL_CIRCUMFERENCE_INCH);

        public static final double POSITION_Kp = .03;
        public static final double POSITION_Ki = 0;
        public static final double POSITION_Kd = 0;
        public static final double POSITION_TOLERANCE = 1;

        public static final double ROTATE_Kp = .022;
        public static final double ROTATE_Ki = 0;
        public static final double ROTATE_Kd = 0;

        public static final double ROTATE_TOLERANCE = 2;

        public static final double MOVE_STRAIGHT_Kp = .01;

        public static final double POSITION_POWER = .5;
        public static final double ROTATE_SPEED = .75;
    }

    public static final class ClawConstants {
        public static final double RIGHT_CLAW_CLOSE_POSITION = .74;
        public static final double RIGHT_CLAW_OPEN_POSITION = .20;
        public static final double LEFT_CLAW_CLOSE_POSITION = .15;
        public static final double LEFT_CLAW_OPEN_POSITION = .70;

    }


}
