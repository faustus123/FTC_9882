/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.util.Set;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="PositionR", group="Linear Opmode")

public class PositionR extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor topDrive = null;
    
    DigitalChannel hook_stop = null;  // Hardware Device Object
    private DcMotor hook = null;
    private Servo hook_lock = null;
    private Servo mascot_launcher = null;
    
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    
    private static final String VUFORIA_KEY = "AZJZ75//////AAABmdAmg40cSEDOrErUXX2lSa2JCItbFqAqd6UYVKvTVWjcw+/gkmtxHQMZL8SwMFpnTmjAzusU1xDqqetDO1iL9KZb0JwlvnurrYtpwJoCx3JyQ+sTQWOcyDA9ciN7KPYizT5idlIpPX+RqwWsNGSSLVMAlWY4Rn1JXojxV5wEAjrpG2lVAKmF2p6nXrpIGJ+FI8HyQjN81Dy3gNIL9crNwZdCQUps6S57KCXETjC1PELLHAWIt3nyYWYgfHo0UNZGzcrKc/0LBw3qDrsXNbDFZiiz29zBkweDjjAkdbYtyii+dHeS6nIk0yopIGqq1YRGxKtK4r4E9Id6jLnqNruNFgChZ1HpfqzMEGBFGL6lDY4q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

        
    BNO055IMU               imu;
    double                  gTheta; // direction to keep IMU facing during GoDir calls
   
       /**
     * Get current angle from gyro
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    
    
    void FirstMotion (int initPos,double scale,double power)
    {
        LowerSelf();
        
        MoveTimed(power, -90, 0.75*scale, true);
        sleep(100);
        
       MoveTimed(power, 150, 0.5*scale, true);
        
       TurnTo( -60, 0.25);
       
        //MoveTimed(power, -90, 3.5*scale, false);
        sleep(500);
        
        MoveTimed(power, 30, 0.5*scale, true);
        int loc = FindGold();
        telemetry.addData("loc", loc);
       telemetry.update();
        
        if (loc == 1) {
            gTheta = -60;
            MoveTimed(power, 120, 4.1*scale, false);
            TurnTo( -120, 0.25);
            MoveTimed(power, 90, 2.1*scale, true);
            TurnTo( 60, 0.25);
            MoveTimed(power,-60, 3.0*scale, true);
            mascot_launcher.setPosition(0.0);
            sleep(2000);
            mascot_launcher.setPosition(1.0);
            sleep(2000);
            
        }
          if (loc == 2) {
            gTheta = -60;
            MoveTimed(power, 82, 3.1*scale, false);
            TurnTo( -60, 0.25);
            MoveTimed(power, 90, 2.5*scale, true);
            TurnTo( 30, 0.25);
            MoveTimed(power, 30, 1.5*scale, true);
            mascot_launcher.setPosition(0.0);
            sleep(2000);
            mascot_launcher.setPosition(1.0);
            sleep(2000);
            
        }
         if (loc == 3) {
            gTheta = -70;
            MoveTimed(power, 60, 3.1*scale, false);
            TurnTo( 30, 0.25);
            MoveTimed(power, 70, 2.5*scale, true);
            TurnTo( 30, 0.25);
            //MoveTimed(power,-60, 3.0*scale, true);
            TurnTo(90, 0.25);
            mascot_launcher.setPosition(0.0);
            sleep(2000);
            mascot_launcher.setPosition(1.0);
            sleep(2000);
            
        }
        telemetry.addData("loc", loc);
       telemetry.update();
        
        
    }
     
    void LowerSelf()
    {
         
        // releqase hook lock
        hook.setPower(-0.4);
        hook_lock.setPosition(0.7);
        sleep(200);    
        
        // Lower robot
        hook.setPower(-0.05);
        sleep(2300);
        hook.setPower(0.0);
        
        // Raise hook slightly
        hook.setPower(0.15);
        sleep(100);
        hook.setPower(0.0);

       }
    

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
        double Tau = gain*(gTheta - getAngle());
        
        double P1 =  Fx*2.0/3.0 + Tau/3.0;
        double P2 = -Fx*1.0/3.0 + 1.0/Math.sqrt(3.0)*Fy + Tau/3.0;
        double P3 = -Fx*2.0/3.0 - 1.0/Math.sqrt(3.0)*Fy + Tau/3.0;

        topDrive.setPower( P1 );
        leftDrive.setPower( P2 );
        rightDrive.setPower( P3 );
    }
    
    void MoveTimed( double power, double theta_degrees, double t_secs, boolean set_gTheta){
        
        // Optionally set member gTheta to current gyro reading so GoDir()
        // will maintain this orientation. Otherwise, use whatever caller left
        if( set_gTheta ) gTheta = getAngle();
        
        double t_start =  getRuntime();
        while ( opModeIsActive() ) {
            
            GoDir( power, theta_degrees );
            
            Position pos = imu.getPosition();
            
            double t_elapsed = getRuntime()-t_start;
            telemetry.addData("T remaining", t_secs-t_elapsed);
            telemetry.addData("delta Theta", getAngle() - gTheta);
            telemetry.addData("encoder", leftDrive.getCurrentPosition());
            telemetry.update();
            if ( t_elapsed >= t_secs ) break;
        }

        topDrive.setPower( 0 );
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );
    }
    
    void TurnTo(double theta_degrees, double power){
        
        double gain =0.017;
        
        while ( opModeIsActive() ) {
            double delta_theta = (theta_degrees - getAngle());
           if (delta_theta < -180) delta_theta += 360;
            if (delta_theta > 180) delta_theta -= 360;
            double P = power * gain * delta_theta;
            if( Math.abs( P ) < 0.1 ) break;
            topDrive.setPower( P );
            leftDrive.setPower( P );
            rightDrive.setPower( P );
        }

        topDrive.setPower( 0 );
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );
    }
    void TriRobotInit()
    {
        telemetry.addData("Status", "initializing...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        topDrive   = hardwareMap.get(DcMotor.class, "pot");
        //arm        = hardwareMap.get(Servo.class, "arm");
        
        hook_stop = hardwareMap.get(DigitalChannel.class, "hook_stop");
        hook  = hardwareMap.get(DcMotor.class, "hook");
        hook_lock  = hardwareMap.get(Servo.class, "hook_lock");
        mascot_launcher  = hardwareMap.get(Servo.class, "mascot_launcher");
        
         hook_stop.setMode(DigitalChannel.Mode.INPUT);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        topDrive.setDirection(DcMotor.Direction.FORWARD);

          
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Status", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        //Remove lock
        hook_lock.setPosition(0.25);
        sleep(1500);
        
        double t_start = getRuntime();
        while( hook_stop.getState() == true ){
        
            double t_diff = getRuntime() - t_start;
        
            if( t_diff > 3 ) break; // only activate motor for 3 seconds max
            hook.setPower(-0.3);
        }
        hook.setPower(0.0);
        
        hook_lock.setPosition(0.0);
        sleep(500);
        //hook_lock.setPosition(0.5);
        //sleep(500);
        //hook_lock.setPosition(0.0);
        //sleep(500);
        
        //small motion to cease rest on the lock
        t_start = getRuntime();
        hook.setPower(0.40);
        sleep(300);
        hook.setPower(0.0);
        
        mascot_launcher.setPosition(0.8);
        sleep(2000);
        mascot_launcher.setPosition(1.0);
        sleep(1000);
    
        // Disable servo PWMs to save power
        ServoImplEx mascotex = hardwareMap.get(ServoImplEx.class, "mascot_launcher");
        ServoImplEx lockex   = hardwareMap.get(ServoImplEx.class, "hook_lock");
        mascotex.setPwmDisable();
        lockex.setPwmDisable();
           
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset(); 
        
    }
    
    @Override
    public void runOpMode() {
       
       TriRobotInit();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int initPos=1;
            double scale=1;
            double power = 0.5;
            gTheta = 0.0;
            //GoSq(power);
            FirstMotion(initPos,scale,power);
            break;
        }
    }
            
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
     private int FindGold2() {
        return 3;}
    private int FindGold() {

          if (tfod != null) {
                tfod.activate();
            }
            
            int pos = 3;
            int Npos2=0;
            int Npos3=0;
            double t_start =  getRuntime();

            while (opModeIsActive()) {
                
                if( (getRuntime() - t_start ) > 3.5) break;
                if (Npos2>=5 || Npos3>=5)break;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      //telemetry.addData("# Object Detected", updatedRecognitions.size());
                      //if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                              if(goldMineralX>600) {
                                  Npos3++;
                              }else if(goldMineralX>300){
                                  Npos2++;
                              }
                             telemetry.addData("gold position", goldMineralX);
                              telemetry.addData("loc", pos);
                             telemetry.update();
                            // sleep(3000);
                             //tfod.shutdown();
                            // return pos;
                             //break;
                           } 
                        }
                      
                    }
                }
            }


        if (tfod != null) {
            tfod.shutdown();
        }
        if (Npos2>=5)return 2;
        if (Npos3>=5)return 3;
        return 1;
    }
    
}

