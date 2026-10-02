package Ex.Software.motor;

import com.qualcomm.robotcore.hardware.DcMotor; // Basic Motor functions
import com.qualcomm.robotcore.util.ElapsedTime; // Elapsed time
import com.qualcomm.robotcore.hardware.DcMotorEx; // Like DcMotor but with extra controlfeatures
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Able to mark for autonomus
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //Need for runOpMode();

@Autonomous


public class Example extends LinearOpMode {  // Cruz's code

    private DcMotorEx m1;
    
    @Override // Overrides runOpMode in parent class
    
    public void runOpMode(){
        
        // Each outlet on the control hub has a changable name
        // Hardware Map (what type of hardware is it, name of it)
        
        m1 = hardwareMap.get(DcMotorEx.class, "motor 1");
        
        ElapsedTime runtime = new ElapsedTime();
        
        //zeros the encoder, sets the tick # back to 0
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Waits for manual start from the driver hub
        waitForStart();
        
        // Sets the velocity of each motor ticks per second
        m1.setVelocity(150);
        
        while(opModeIsActive() && runtime.seconds() < 10.0){
            
        }
        
        m1.setVelocity(0);
        
        /*if(runtime.seconds()>10.0){   // This will work too
            motor.setVelocity(0);        
            
        } */
        
        
    }
}
/*
public class BenMT extends LinearOpMode{  // Ben's code

    private DcMotorEx m1;

    @Override
    
    public void runOpMode(){
        
    m1 = hardwareMap.get(DcMotorEx.class, "motor 1");
    m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    
    waitForStart();
    
    m1.setVelocity();
    
    sleep(5000);
    
    m1.setVelocity(1);

    sleep(5000);
    
    m1.setVelocity(1);
    }

}
*/

/*
public class SamirMT extends LinearOpMode { // Samir's code

    private DcMotorEx m1;
    int vel = 150;
    int stop = 0;
        
    @Override // Overrides runOpMode in parent class

    public void runOpMode() {
        // // Initialize motors
        // DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "left_motor");
        // DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "right_motor");

        // // Set motor directions
        // leftMotor.setDirection(DcMotor.Direction.FORWARD);
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // // Set motor power
        // leftMotor.setPower(0.5);
        // rightMotor.setPower(0.5);

        // // Wait for a while
        // sleep(2000);

        // // Stop motors
        // leftMotor.setPower(0);
        // rightMotor.setPower(0);
        
        // Each outlet on the cotrol hub has a changable name
        // Hardware Map (hardware type & name)

        m1 = hardwareMap.get(DcMotorEx.class, "motor 1");

        ElapsedTime runtime = new ElapsedTime();

        //zeroes the encoder, sets the tick # back to 0
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Waits for the manual start from the driver hub
        waitForStart();

        // Sets the velocity of each motor ticks per second
        m1.setVelocity(vel);

        while(opModeIsActive() && runtime.seconds() < 10) {
            if(runtime.seconds() >= 5) {
                
                switchMotorDirection(m1);
            }
        }

        m1.setVelocity(stop); // Stops the motor
    }

    public void switchMotorDirection(DcMotorEx motor) {
        vel = -vel; // Reverses the velocity
        motor.setVelocity(vel); // Sets the new velocity
    }
}
*/

/*

public class EthanMT extends LinearOpMode {  // Ethan's code

    private DcMotorEx m1;

    @Override // Overrides runOpMode in parent class

    public void runOpMode(){

        //Each outlet on the control hub has a changable name
        //Hardeware Map (what type of hardware is it, name of it)

        m1 = hardwareMap.get(DcMotorEx.class, "motor 1");

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        m1.setVelocity(200);

        while(opModeIsActive() && runtime.seconds() < 10.0){

        }

        m1.setVelocity(0);
        
        while(opModeIsActive() && runtime.seconds() < 15.0 ){

        }

        m1.setVelocity(-200);

        while(opModeIsActive() && runtime.seconds() < 25.0){

        }

        m1.setVelocity(0);
   
    }
}
*/
/*

public class motortest extends LinearOpMde {  // Ezra's code
    private DcMotorEx m1;

    @Override

    public void runOpMode(){
        m1 = hardwareMap.get (DcMotorEx.class, "motor 1");

        ElapsedTIm runtime = new ElapsedTime();

        m1.setMode(DcMotor.RunMode.Stop_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        m1.setVelocity(150);

        while(opModeIsActive() && runtime.seconds() <11.0){

        }

        m1.setVelocity(0);

         while(opModeIsActive() && runtime.seconds() >18.0){

        }

         m1.setVelocity(-150);
         while(opModeIsActive() && runtime.seconds() <25.0){

        }

        m1.setVelocity(0);


        

    }
}

*/
