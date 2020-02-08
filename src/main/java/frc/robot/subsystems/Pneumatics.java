package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {

    private Joystick DriverStick;
    private Solenoid s1, s2;
    private Compressor airCompressor;

    public void airsystem() {

        DriverStick = new Joystick(1);
        airCompressor = new Compressor(1);
        airCompressor.start();

        s1 = new Solenoid(1);
        s2 = new Solenoid(2);
    }

    public void autonomous() {
    }

    public void operatorControl() {
        if (DriverStick.getRawButton(1) == true)
            ;
        {
            s1.set(true);
            s2.set(false);
        }
        if (DriverStick.getRawButton(2) == true)
            ;
        {
            s1.set(false);
            s2.set(true);

        }
    }

    public void deployStage1() {
        //TODO:send pneumatics up with climber
        //TODO: make sure ddeploy in correct direction
    }

    public void deployStage2() {
    }

    public void retract1() {
        //TODO: lower climber so we can try again
    }

    public void retract2() {
    }

    public void stow() {
        //TODO: fully retract climbing gear
    }

    public void reversePneumatics() {
        //TODO: reverse the pneumatics for ascent
    }
}
