package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Testauto1 extends LinearOpMode {
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    
    @Override
    public void runOpMode (){
        leftmotor = hardwareMap.get (DcMotor.class, "left");
        telemetry.addData("Status", "Initialized" );
        telemetry.update();
       
        waitForStart();
        double tgtPower = 1;
        while (opModeIsActive()) {
           leftmotor.setPower (tgtPower);
           telemetry.addData ("Target Power",tgtPower);
           telemetry.addData("Status", "Running" );
           telemetry.addData ( "Motor Power", leftmotor.getPower());
            telemetry.update(); 
            
        }
        //leftmotor.setPower (0);
        
        
        
    }

    // todo: write your code here
}