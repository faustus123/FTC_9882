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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.util.Set;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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
@Autonomous(name="DDtest", group="Linear Opmode")

public class DDtest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor topDrive = null;
    private Servo   arm      = null;

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
    void GoSq( double power){
        gTheta = 0.0;
     
        MoveTimed( power,  0.0, 2.0, false);
        sleep(2000);
        TurnTo( 90., power);
        sleep(2000);
        MoveTimed( power,  0.0, 2.0, true);
        sleep(2000);
        TurnTo( 180., power);
        sleep(2000);
        MoveTimed( power,  0.0, 2.0, true);
        //sleep(2000);
        
        //while(true){    
            telemetry.addData("delta Theta", getAngle());
            telemetry.update();
        //}
        
        TurnTo( 270., power);
        sleep(2000);
           // MoveTimed( power,  270.0, 2.0, true);
            /*sleep(2000);
            TurnTo( 0., power);*/
            //telemetry.addData(delta_theta < -180) delta_theta += 360;
    }
    
        void FirstMotion (int initPos,double scale,double power){
        if(initPos==1)
        {
            MoveTimed(power, 0, 4.*scale, false);
            sleep(500);
            telemetry.addData("arm", arm.getPosition()); 
            telemetry.update();
        }
        else if(initPos==2)
        {
            MoveTimed(power, 0, 1.2*scale, false);
            sleep(500);
             MoveTimed(power, 45, 3*scale, false);
            sleep(500);
             MoveTimed(power, 135, 4*scale, false);
            sleep(500);
            
            
            
        }
        if(arm.getPosition()<.5)
        {
            arm.setPosition(1.0);
        }
        else
        {
            arm.setPosition(0.0);
        }
        sleep(5000);
        telemetry.update();
        
        if(initPos==1)
        {
        MoveTimed(power, -90, 3.5*scale, false);
        sleep(500);   
        }
        else if(initPos==2)
        {
        MoveTimed(power, 0, 3.5*scale, false);
        sleep(500); 
        }
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
            if( Math.abs( P ) < 0.08 ) break;
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
        arm        = hardwareMap.get(Servo.class, "arm");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset(); 
    }
    
    @Override
    public void runOpMode() {
       int numWheels=3;
       if(numWheels==3)
       {
        TriRobotInit();
       }
       else if(numWheels==4)
       {
           //QuadRobotInit();
       }
       
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int initPos=2;
            double scale=.25;
            double power = 0.5;
            gTheta = 0.0;
            //GoSq(power);
            FirstMotion(initPos,scale,power);
            break;
        }
    }
}
