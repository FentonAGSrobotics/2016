package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a TigerTechBot.
 * See TigerTechBotTeleopTank_Iterative and others classes starting with "TigerTechHardware" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are CamelCase with no spaces.
 *
 * Motor channel:  Left  drive motor:                   "LeftDriveMotor1"
 * Motor channel:  Right drive motor:                   "RightDriveMotor2"
 * Motor channel:  Left flywheel motor:                 "LeftFlywheel"
 * Motor channel:  Right flywheel motor:                "RightFlywheel"
 * Motor channel:  Intake motor:                        "IntakeMotor"
 * Motor channel:  Elevator motor                       "ElevatorMotor"
 * Servo channel:  Servo to operate left cap ball arm:  "LeftCapballServo"
 * Servo channel:  Servo to operate right cap ball arm: "RightCapballServo"
 * Servo channel:  Servo beacon button pusher:          "BeaconPusher"
 * Sensor channel: Light sensor 1:                      "LightSensor1"
 * Sensor channel: Touch sensor 1:                      "TouchSensor1"
 */
public class TigerTechHardware
{
    /* Public OpMode members. */
    public DcMotor  leftMotor       = null;
    public DcMotor  rightMotor      = null;
    public DcMotor  leftShooter     = null;
    public DcMotor  rightShooter    = null;
    public DcMotor  liftMotor       = null;
    public DcMotor  intakeMotor     = null;
    public Servo    leftCapServo    = null;
    public Servo    rightCapServo   = null;
    public Servo    buttonPusher    = null;

    public static final double MID_SERVO        =  0.5 ;     //update
    public static final double ARM_OPEN_POWER   =  0.45 ;    //update
    public static final double ARM_CLOSE_POWER  = -0.45 ;    //update

    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TigerTechHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor       = hwMap.dcMotor.get("LeftDriveMotor1");
        rightMotor      = hwMap.dcMotor.get("RightDriveMotor1");
        leftShooter     = hwMap.dcMotor.get("LeftFlywheel");
        rightShooter    = hwMap.dcMotor.get("RightFlywheel");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftShooter.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightShooter.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        liftMotor.setPower(0);
        intakeMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftCapServo    = hwMap.servo.get("LeftCapballServo");
        rightCapServo   = hwMap.servo.get("RightCapballServo");
        buttonPusher    = hwMap.servo.get("BeaconPsuher");
        leftCapServo.setPosition(MID_SERVO);
        rightCapServo.setPosition(MID_SERVO);
        buttonPusher.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

