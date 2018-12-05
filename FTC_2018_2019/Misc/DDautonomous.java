package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@Autonomous

public class DDautonomous extends LinearOpMode {

    DigitalChannel hook_stop = null;  // Hardware Device Object
    private DcMotor hook = null;
    private Servo hook_lock = null;
    private Servo mascot_launcher = null;

    @Override
    public void runOpMode() {
        
        // get a reference to our digitalTouch object.
        hook_stop = hardwareMap.get(DigitalChannel.class, "hook_stop");
        hook  = hardwareMap.get(DcMotor.class, "hook");
        hook_lock  = hardwareMap.get(Servo.class, "hook_lock");
        mascot_launcher  = hardwareMap.get(Servo.class, "mascot_launcher");
        
        // set the digital channel to input.
        hook_stop.setMode(DigitalChannel.Mode.INPUT);
        
        // Init -------------------------------------
        telemetry.addData("initializing", "...");
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
            hook.setPower(-0.5);
        }
        hook.setPower(0.0);
        
        // Activate hook lock
        t_start = getRuntime();
        hook_lock.setPosition(0.0);
        while( (getRuntime() - t_start) < 0.5 );

        // Take up slack in hook
        t_start = getRuntime();
        hook.setPower(0.4);
        while( (getRuntime() - t_start) < 0.3 );
        hook.setPower(0.0);
        
        //--------------------------------------------------        
        // wait for the start button to be pressed.
        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();

        // Lift robot slightly to release pressure on hook_lock
        // so it can be released
        t_start = getRuntime();
        hook.setPower(-0.4);
        hook_lock.setPosition(0.7);
        while( (getRuntime() - t_start) < 0.2 );
        
        // Gently lower bot by applying just enough power so that gravity
        // is still stronger
        t_start = getRuntime();
        hook.setPower(-0.05);
        while( (getRuntime() - t_start) < 3.0 );
        hook.setPower(0.0);
        
        // Wait for 1 second (just so I can observe separate motions)
        t_start = getRuntime();
        while( (getRuntime() - t_start) < 0.5 );
        
        // At this point there is still pressure on the hook.
        // raise it slightly in order to clear the handle
        t_start = getRuntime();
        hook.setPower(0.25);
        while( (getRuntime() - t_start) < 0.2 );
        hook.setPower(0.0);
   }
    
}

