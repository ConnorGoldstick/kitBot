///*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.-
 */
public class CentralCode extends IterativeRobot {

    Jaguar jagleft, jagright;
    Joystick xBox;
    Victor victor;
    Solenoid sol1, sol2, sol4, sol5, sol7, sol8;
    Relay relay, compressor;
    DigitalInput digi14, digi13, digi3;
    AnalogChannel ultrasonic, encoder;
    Gyro gyro;
    double conf;
    boolean atShoot, checkGyro, afterShoot, inRange, tooClose, tooFar;
    int endTimer, noWait, gyroTimer;
    NetworkTable server = NetworkTable.getTable("smartDashboard");
    Drive drive;
    loadAndShoot loadAndShoot;
    SmartDashboard smart;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        jagleft = new Jaguar(1);
        jagright = new Jaguar(2);
        victor = new Victor(3);

        sol1 = new Solenoid(1);
        sol2 = new Solenoid(2);

        sol4 = new Solenoid(4);
        sol5 = new Solenoid(5);

        sol7 = new Solenoid(7);
        sol8 = new Solenoid(8);

        relay = new Relay(1);
        compressor = new Relay(2);

        digi14 = new DigitalInput(14);
        digi13 = new DigitalInput(13);
        digi3 = new DigitalInput(3);

        encoder = new AnalogChannel(2);
        ultrasonic = new AnalogChannel(3);

        gyro = new Gyro(1);
        gyro.setSensitivity(0.07);
        gyro.reset();

        xBox = new Joystick(1);

        conf = 0;
        noWait = 0;
        endTimer = 0;
        noWait = 0;
        gyroTimer = 0;

        atShoot = false;
        afterShoot = false;
        checkGyro = true;

        drive = new Drive(jagleft, jagright, sol1, sol2, xBox);
        loadAndShoot = new loadAndShoot(encoder, victor, sol4, sol5, sol7, sol8, xBox, digi14, digi13, digi3, smart);
        drive.start();
        loadAndShoot.start();
        
        compressor.set(Relay.Value.kOn);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousInit() {
        compressor.set(Relay.Value.kOn);
        
        gyro.reset();
        endTimer = 0;
        conf = 0;
        relay.set(Relay.Value.kOn);
        noWait = 0;
        gyroTimer = 0;

        sol1.set(true); //change it to fast setting
        sol2.set(false);
        sol4.set(false);
        sol5.set(true);
        sol7.set(true);
        sol8.set(false);

        atShoot = false;
        afterShoot = false;
        checkGyro = false;
    }

    public void autonomousPeriodic() {
        compressor.set(Relay.Value.kOn);
        relay.set(Relay.Value.kOn);
        System.out.println("Confidence: " + conf);
        if (!checkGyro && !atShoot) { //if program does not know it's in range, do the following
            if (ultrasonic.getVoltage() > 0.86) { //if not in range, do the following
                conf = conf + SmartDashboard.getNumber("Confidence") - 70; //add to the total confidence
                jagleft.set(-0.648); //move towards the goal
                jagright.set(0.6);
                System.out.println("Driving forwards.");
            } else { //once in range, do the follwing
                jagleft.set(0); //stop moving forwards
                jagright.set(0);
                checkGyro = true; // tell the program to check the gyro
                System.out.println("Checking gyro.");
            }
        }
        if (checkGyro) {
            gyroTimer++;
            if (gyro.getAngle() < -2) { //if the robot is pointed to the left, do the following
                jagleft.set(-0.108); //turn right
                jagright.set(-0.1);
                System.out.println("Orienting right.");
            }
            if (gyro.getAngle() > 2) { //if the robot is pointed to the right, do the following
                jagleft.set(0.108); //turn right
                jagright.set(0.1);
                System.out.println("Orienting left.");
            }
            if (gyro.getAngle() > -2 && gyro.getAngle() < 2) { // if the robot is pointed towards the goal, do the following
                jagleft.set(0); //stop the motion of the robot
                jagright.set(0);
                checkGyro = false; //stop looking at the gyro
                atShoot = true; //tell the program that the robot is in position
                System.out.println("Oriented.");
            }
            if (gyroTimer == 30) { //after three fifths second of checking the gyro, do the following
                jagleft.set(0); //stop the motion of the robot
                jagright.set(0);
                checkGyro = false; //stop looking at the gyro
                atShoot = true; //tell the program that the robot is in position
                System.out.println("Gyro check timed out.");
            }
        }
        if (atShoot && !afterShoot) { //once in position, do the following
            if (conf >= 40) { //if the target has been seen, do the following
                System.out.println("Saw Target.");
                sol7.set(true); //launch the catapult, switched these??????????????????????
                sol8.set(false);
                afterShoot = true; //tell the program it has fired
                System.out.println("Launching.");
            }
            if (conf < 40) { //if the target has not been seen, do the following
                if (noWait == 0) { //reset the timer for this occasion
                    System.out.println("Did not see target.");
                }
                noWait++; //count the timer up
                if (noWait == 200) { //once the rimer reaches 4 seconds, do the following
                    sol7.set(true); //launch the catapult, switched these??????????????????
                    sol8.set(false);
                    afterShoot = true; //tell the program it has fired
                    System.out.println("Launching.");
                }
            }
        }
        if (afterShoot) { //once the program knows it has fired, do the following
            if (endTimer < 100) { // for two seconds after firing, do the following
                endTimer++; //run the ending timer
                jagleft.set(0); //stop any motion of the robot
                jagright.set(0);
                if (endTimer == 100) { //at end of autonomous, do the following
                    System.out.println("Autonomous Complete.");
                }
            }
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopInit() {
        compressor.set(Relay.Value.kOn);
        
        gyro.reset();
        relay.set(Relay.Value.kOff);
        drive.setRun(true);
        loadAndShoot.setRun(true);
    }

    public void teleopPeriodic() {
        compressor.set(Relay.Value.kOn);
        
        smart.putBoolean("fast gear", sol1.get() == true && sol2.get() == false);
        smart.putBoolean("slow gear", sol1.get() == false && sol2.get() == true);

        if (gyro.getAngle() > 360 || gyro.getAngle() < -360) {
            gyro.reset();
        }
        smart.putNumber("Angle", gyro.getAngle());

        if (ultrasonic.getVoltage() < 0.5) {
            tooClose = true;
        } else {
            tooClose = false;
        }
        smart.putBoolean("Too close", tooClose);

        if (ultrasonic.getVoltage() > 1) {
            tooFar = true;
        } else {
            tooFar = false;
        }
        smart.putBoolean("Too far", tooFar);

        if (ultrasonic.getVoltage() >= 0.5 && ultrasonic.getVoltage() <= 1) {
            inRange = true;
        } else {
            inRange = false;
        }
        smart.putBoolean("In range", inRange);
    }

    public void disabledInit() {
        drive.setRun(false);
        loadAndShoot.setBooleansToZero();
        loadAndShoot.setRun(false);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() { //charge the compressor
        compressor.set(Relay.Value.kOn);
    }
}
