#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Solenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <cameraserver/CameraServer.h>
#include <wpi/raw_ostream.h>
#include <frc/DigitalInput.h>

#define __linux__

using namespace frc;

// Variable declaration //

// DRIVER //
WPI_TalonSRX tln_leftWheel { 1 }; //left motor
WPI_TalonSRX tln_rightWheel { 2 }; //right motor

Encoder enc_leftWheel { 0, 1 }; //left wheel encoder
Encoder enc_rightWheel { 2, 3 }; //right wheel encoder

DifferentialDrive drv_wheels { tln_leftWheel, tln_rightWheel }; //left + right wheel diff. drive
const double accelMultMin = 0.3; //minimum for the acceleration multiplier
double accelMult = accelMultMin; //used to modify acceleration multiplier for the wheels
const double accelMultMax = 0.75; //max for the acceleration multiplier

Joystick joy_driver { 0 }; //controller for main driver
//axis and button values on the controller
int driver_leftStickY = 1, driver_rightStickY = 5;

// CO-DRIVER //
WPI_TalonSRX tln_arm { 5 }; //arm connected to Cargo intake box
const double armVelMult = 0.25;

DigitalInput limiterSwitchBottom { 4 }, limiterSwitchTop { 5 };

WPI_VictorSPX vic_cargoIO { 7 }; //axel + wheels used for intake and output of cargo balls

Joystick joy_codriver { 1 }; //controller for co driver (Arm control)
//axis and button values on the controller
int co_stickY = 1, co_cargoPowerToggle = 2, co_cargoIOToggle = 3;

// MISCELLANEOUS //
const double pi = 3.1415926535897; //used for encoder distances

// End variable declaration //

void Robot::RobotInit() {
  #if defined(__linux__)
    CameraServer::GetInstance()->StartAutomaticCapture();
  #else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
  #endif

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  SmartDashboard::PutData("Auto Modes", &m_chooser);

  enc_rightWheel.SetSamplesToAverage(5); 
  enc_rightWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);
  enc_rightWheel.SetMinRate(1.0);

  enc_leftWheel.SetSamplesToAverage(5);
  enc_leftWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);
  enc_leftWheel.SetMinRate(1.0);

  tln_arm.SetNeutralMode(NeutralMode::Brake);
  tln_arm.SetInverted(true);
  
  SmartDashboard::PutString("Cargo Intake Direction: ", (vic_cargoIO.GetInverted() ? "In" : "Out"));
}

void Robot::RobotPeriodic() {

  //checks if the sticks are pushed forward or backward
  bool sticksMovingY = joy_driver.GetRawAxis(driver_leftStickY) > 0.1 || joy_driver.GetRawAxis(driver_rightStickY) > 0.1 || 
     joy_driver.GetRawAxis(driver_leftStickY) < -0.1 || joy_driver.GetRawAxis(driver_rightStickY) < -0.01;

  if(sticksMovingY && accelMult < accelMultMax) { //one of the sticks were moved
    accelMult += 0.01; //add 0.01 to the acceleration multiplier
  } else if(sticksMovingY && accelMult >= accelMultMax) { //the acceleration multiplier has reached its maximum value
    accelMult = accelMultMax;
  } else { //the sticks aren't being pushed
    accelMult = accelMultMin; //reset the acceleration multiplier
  }

  SmartDashboard::UpdateValues();
}

void Robot::AutonomousInit() {

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    //Default Auto goes here
  }

}

void Robot::AutonomousPeriodic() {

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    while(enc_leftWheel.GetDistance() < 10) { //the left wheel's distance is less than 10
      //set the drive to 0.5 speed
      drv_wheels.TankDrive(0.5, 0.5);
    }

    //stop the drive
    drv_wheels.TankDrive(0, 0);
  }

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //tank drive that multiplies by the acceleration multiplier (determined in RobotPeriodic())
  drv_wheels.TankDrive(joy_driver.GetRawAxis(driver_leftStickY) * accelMult, joy_driver.GetRawAxis(driver_rightStickY) * accelMult);

  //neither limiter switch is hit
  if (!limiterSwitchBottom.Get() && !limiterSwitchTop.Get()) {
    //move the arm
    tln_arm.Set(joy_codriver.GetRawAxis(co_stickY) * armVelMult);
  } 
  //the bottom limiter switch is hit and the joystick is pushed forward
  else if(limiterSwitchBottom.Get() && joy_codriver.GetRawAxis(co_stickY) > 0) { 
    //move the arm
    tln_arm.Set(joy_codriver.GetRawAxis(co_stickY) * armVelMult);
  } 
  //the top limiter switch is hit and the joystick is pulled backward
  else if(limiterSwitchTop.Get() && joy_codriver.GetRawAxis(co_stickY) < 0) {
    //move the arm
    tln_arm.Set(joy_codriver.GetRawAxis(co_stickY) * armVelMult);
  }

  if(joy_codriver.GetRawButtonPressed(co_cargoPowerToggle)) {
    //set the cargo motor off if it's on, or on if it's off.
    vic_cargoIO.Get() > 0.0 ? vic_cargoIO.Set(0.0) : vic_cargoIO.Set(0.5);
  }

  if(joy_codriver.GetRawButtonPressed(co_cargoIOToggle)) {
    //set the cargo motor inverted if it's not, or not if it is.
    vic_cargoIO.GetInverted() ? vic_cargoIO.SetInverted(false) : vic_cargoIO.SetInverted(true);
  }

  SmartDashboard::UpdateValues();
}

//Use these functions to test controls BEFORE putting them in periodic classes.
void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return StartRobot<Robot>(); }
#endif
