package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.List;

@Autonomous

public class DDautonomous extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor topDrive = null;

    DigitalChannel hook_stop = null;  // Hardware Device Object
    private DcMotor hook = null;
    private Servo hook_lock = null;
    private Servo mascot_launcher = null;

    BNO055IMU               imu;
    
    private double gTheta = 0;
    private boolean enableTorqueCorrection = true; // used to maitain gryo angle during movement

    @Override
    public void runOpMode() {
        
        // get a reference to hardware
        leftDrive        = hardwareMap.get(DcMotor.class, "left");
        rightDrive       = hardwareMap.get(DcMotor.class, "right");
        topDrive         = hardwareMap.get(DcMotor.class, "pot");
        hook_stop        = hardwareMap.get(DigitalChannel.class, "hook_stop");
        hook             = hardwareMap.get(DcMotor.class, "hook");
        hook_lock        = hardwareMap.get(Servo.class, "hook_lock");
        mascot_launcher  = hardwareMap.get(Servo.class, "mascot_launcher");
        
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        topDrive.setDirection(DcMotor.Direction.REVERSE);
        
        
        // set the digital channel to input.
        hook_stop.setMode(DigitalChannel.Mode.INPUT);
        
        // Init -------------------------------------
        telemetry.addData("initializing", "...");
        telemetry.update();
 
        // Calibrate gyro
        telemetry.addData("Status", "calibrating gyro...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imu_par = new BNO055IMU.Parameters();
        imu_par.mode                = BNO055IMU.SensorMode.IMU;
        imu_par.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu_par.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_par.loggingEnabled      = false;
        imu.initialize(imu_par);
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
       
        // Reposition hook to starting position
        telemetry.addData("Status", "initializing hook ...");
        telemetry.update();
       
        // Free hook lock
        double t_start = getRuntime();
        hook_lock.setPosition(0.7);
        while( (getRuntime() - t_start) < 0.5 );
        
        // Move hook down to starting position. Use limit switch to public void stop
        // motion or time as a backup in case switch malfunctions
        t_start = getRuntime();
        while( hook_stop.getState() == true ){
        
            double t_diff = getRuntime() - t_start;
            telemetry.addData("Run time", t_diff);
            telemetry.update();
        
            if( t_diff > 3 ) break; // only activate motor for 3 seconds max
            hook.setPower(-0.3);
        }
        hook.setPower(0.0);
        
        // Activate hook lock
        t_start = getRuntime();
        hook_lock.setPosition(0.0);
        while( (getRuntime() - t_start) < 0.5 );

        // Take up slack in hook
        t_start = getRuntime();
        hook.setPower(0.3);
        while( (getRuntime() - t_start) < 0.3 );
        hook.setPower(0.0);
        
        // Exercise mascot placer
        for( double pos = 1.0; pos>-0.25; pos -=0.25){
            telemetry.addData("Status", "mascot launcher ...");
            telemetry.addData("position", pos);
            telemetry.update();
            mascot_launcher.setPosition(pos);
            sleep(250);
        }
        mascot_launcher.setPosition(1.0);
        sleep(750);
        
        // Disable servo PWMs to save power
        ServoImplEx mascotex = hardwareMap.get(ServoImplEx.class, "mascot_launcher");
        ServoImplEx lockex   = hardwareMap.get(ServoImplEx.class, "hook_lock");
        mascotex.setPwmDisable();
        lockex.setPwmDisable();
        
        //--------------------------------------------------        
        // wait for the start button to be pressed.
        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();

        if(false){
        // Move straight forward to calibrate left and right motors
        double pos_left  =  leftDrive.getCurrentPosition();
        double pos_right = rightDrive.getCurrentPosition();
        double pos_top   = topDrive.getCurrentPosition();
        enableTorqueCorrection = false;
        //MoveTimed(0.5, 90.0, 5.0, false);
        MoveTimed(0.5, 210.0, 5.0, false);
        double delta_left  =  leftDrive.getCurrentPosition() - pos_left;
        double delta_right = rightDrive.getCurrentPosition() - pos_right;
        double delta_top   =   topDrive.getCurrentPosition() - pos_top;
        double ratio_lr = delta_left/delta_right;
        double ratio_rt = delta_right/delta_top;
        telemetry.addData("left/right", ratio_lr);
        telemetry.addData("right/top", ratio_rt);
        telemetry.addData("angle", getAngle());
        telemetry.update();
        }
        
        //enableTorqueCorrection = false;
        MoveTimed(0.5, 0.0, 1.5, false);
        MoveTimed(0.5, 90.0, 1.5, false);
        MoveTimed(0.5, 180.0, 1.5, false);
        MoveTimed(0.5, 270.0, 1.5, false);
        
        sleep(10000);
    }
    
    //---------------------
    // getAngle
    //---------------------
    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //---------------------
    // GoDir
    //---------------------
    void GoDir( double power, double theta_degrees ){
        // Set motors to move the robot in the direction given
        // by theta_degrees with the given power. The motion will
        // try and maintain the gyro-Z angle given by the theta_face
        // member during the move by adjusting the Torque value.
        //
        // The intent of this is to simply set the motors and return
        // immediately. This should be called from within a loop that
        // breaks when it decides the movement is done.
        
        double gain = 0.1; // torque power per degree angle
        
        double theta_radians = Math.toRadians(theta_degrees);
        double Fx = power * Math.cos( theta_radians );
        double Fy = power * Math.sin( theta_radians );
        double Tau = -gain*(gTheta - getAngle());
        
        // Optionally turn off the torque correction used to maintain angle
        if( !enableTorqueCorrection ) Tau = 0;
        
        double P1 =  Fx*2.0/3.0 + Tau/3.0;
        double P2 = -Fx*1.0/3.0 + 1.0/Math.sqrt(3.0)*Fy + Tau/3.0;
        double P3 = -Fx*2.0/3.0 - 1.0/Math.sqrt(3.0)*Fy + Tau/3.0;
        
        // Calibration parameters obtained empirically
        P3 *= 1.2;
        P1 *= 1.6;
        
        // Make sure no value is outside range -1,1, by scaling all values
        // if necessary.
        double scale = 1.0;
        if( scale*Math.abs(P1) > 1.0 ) scale *= 1.0/Math.abs(P1);
        if( scale*Math.abs(P2) > 1.0 ) scale *= 1.0/Math.abs(P2);
        if( scale*Math.abs(P3) > 1.0 ) scale *= 1.0/Math.abs(P3);
        
        P1 *= scale;
        P2 *= scale;
        P3 *= scale;

        topDrive.setPower( P1 );
        leftDrive.setPower( P2 );
        rightDrive.setPower( P3 );
    }
    
    //---------------------
    // MoveTimed
    //---------------------
    void MoveTimed( double power, double theta_degrees, double t_secs, boolean set_gTheta){
        
        // Optionally set member gTheta to current gyro reading so GoDir()
        // will maintain this orientation. Otherwise, use whatever caller has
        // set gTheta to
        if( set_gTheta ) gTheta = getAngle();
        
        double pos_left  =  leftDrive.getCurrentPosition();
        double pos_right = rightDrive.getCurrentPosition();
        double pos_top   =   topDrive.getCurrentPosition();
        
        double t_start =  getRuntime();
        while ( opModeIsActive() ) {
            
            GoDir( power, theta_degrees );
            
            Position pos = imu.getPosition();
            
            double t_elapsed = getRuntime()-t_start;
            telemetry.addData("T remaining", t_secs-t_elapsed);
            telemetry.addData("delta Theta", getAngle() - gTheta);
            telemetry.addData("left",   leftDrive.getCurrentPosition() - pos_left);
            telemetry.addData("right", rightDrive.getCurrentPosition() - pos_right);
            telemetry.addData("top",     topDrive.getCurrentPosition() - pos_top);
            telemetry.update();
            if ( t_elapsed >= t_secs ) break;
        }

        topDrive.setPower( 0 );
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );
    }
    
    //---------------------
    // TurnTo
    //
    // power - Over scale for motor power. Power will be set proportional to
    //         difference in angle (with cutoff)
    //
    // tol   - Tolerance in degrees. Routine will not return until measured
    //         and target angle are within this limit.
    //---------------------
    void TurnTo(double theta_degrees, double power, double tol){
        
        double gain =0.017;
        
        while ( opModeIsActive() ) {
            double delta_theta = (theta_degrees - getAngle());
            if( Math.abs(delta_theta) < tol ) break;

            if (delta_theta < -180) delta_theta += 360;
            if (delta_theta > 180) delta_theta -= 360;
            double P = power * gain * delta_theta;
            if( Math.abs( P ) < 0.1 ) P = 0.1*P/Math.abs( P );
            topDrive.setPower( P );
            leftDrive.setPower( P );
            rightDrive.setPower( P );
        }

        topDrive.setPower( 0 );
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );
    }
}


