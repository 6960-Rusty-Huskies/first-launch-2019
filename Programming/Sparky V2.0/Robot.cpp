#include "Robot.h"

#include <iostream>
#include <thread>

#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <ctre/phoenix/motorcontrol/GroupMotorControllers.h>
#include <ctre/Phoenix.h>

#include <cameraserver/CameraServer.h>

#include <frc/AnalogInput.h>
#include <frc/Compressor.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/SpeedControllerGroup.h>

#include <wpi/raw_ostream.h>

using namespace frc;

// Variable declaration //

// DRIVER //

//left motors
WPI_TalonSRX tln_leftFront { 1 };
WPI_TalonSRX tln_leftBack { 2 };
SpeedControllerGroup grp_left { tln_leftFront, tln_leftBack };

//right motors
WPI_TalonSRX tln_rightFront { 3 };
WPI_TalonSRX tln_rightBack { 4 };
SpeedControllerGroup grp_right { tln_rightFront, tln_rightBack };

//left wheel encoder
Encoder enc_leftWheel { 0, 1 };
//right wheel encoder
Encoder enc_rightWheel { 2, 3 };

//left + right wheel diff. drive
DifferentialDrive drv_wheels { grp_left, grp_right };

//actuator wheel
WPI_VictorSPX vic_actuatorBack { 6 };

//controller for main driver
Joystick joy_driver { 0 };
//axis and button values on the controller
int driver_leftStickY = 1, driver_rightStickY = 5;

// CO-DRIVER //

//arm connected to Cargo intake box
WPI_TalonSRX tln_arm { 5 };
//multiplier for speed of the arm
const double armVelMult = 0.75;

DigitalInput limiterSwitchTop { 4 }, limiterSwitchBottom { 5 };

//axel + wheels used for intake and output of cargo balls
WPI_VictorSPX vic_cargoIO { 7 };

//controller for co driver (Arm control)
Joystick joy_codriver { 1 };
//axis and button values on the controller
int co_stickY = 1, co_cargoPowerToggle = 1, co_cargoIOToggle = 2;

// MISCELLANEOUS //

//used for encoder distances
const double pi = 3.1415926535897;

// End variable declaration //

void Robot::RobotInit() {

  // We need to run our vision program in a separate thread. If not, our robot
  // program will not run.
  std::thread visionThread(VisionThread);
  visionThread.detach();
  
  //Set autonomous mode choice on the smart dashboard.
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Sets the number of the samples the encoder gets to average when calculating period.
  enc_rightWheel.SetSamplesToAverage(5); 
  //Used to calculate distance traveled in one rotation. Distance in this case is calculated in inches per second. 
  //Formula: 1.0 / 360.0 * 2.0 * pi * radius
  enc_rightWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);
  //Sets the minimum number of inches per second for the robot to be considered to be moving.
  enc_rightWheel.SetMinRate(1.0);

  //Repeat the above process for the left wheel encoder.
  enc_leftWheel.SetSamplesToAverage(5);
  enc_leftWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);
  enc_leftWheel.SetMinRate(1.0);

  //Set the talon arm to Brake Mode.
  tln_arm.SetNeutralMode(NeutralMode::Brake);
  tln_arm.SetInverted(true);

  //Set the left wheel inverted so it matches with the right one.
  grp_left.SetInverted(true);

  //Set the right back talon inverted so it doesn't move 
  //in the opposite direction of the other one.
  tln_rightBack.SetInverted(true);

  //Put some important values on the Smart Dashboard.
  //UpdateDashboardValues();
  SmartDashboard::UpdateValues();
}


void Robot::RobotPeriodic() {
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
     //the left wheel's distance is less than the maximum
    while(enc_leftWheel.GetDistance() < 10) {
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
  drv_wheels.TankDrive(joy_driver.GetRawAxis(driver_leftStickY) , 
                       joy_driver.GetRawAxis(driver_rightStickY) );

  //neither limiter switch is hit, or 
  //the bottom switch is hit and the stick is moving back, or 
  //the top switch is hit and the stick is pressed forward
  if ((!limiterSwitchBottom.Get() && !limiterSwitchTop.Get()) || 
      (limiterSwitchBottom.Get() && joy_codriver.GetRawAxis(co_stickY) > 0) || 
      (limiterSwitchTop.Get() && joy_codriver.GetRawAxis(co_stickY) < 0)) {
    //move the arm
    tln_arm.Set(joy_codriver.GetRawAxis(co_stickY) * armVelMult);
  }

  if(joy_codriver.GetRawButtonPressed(co_cargoPowerToggle)) {
    //set the cargo motor off if it's on, or on if it's off.
    vic_cargoIO.Set(vic_cargoIO.Get() > 0.0 ? 0.0 : 0.5);
  }

  if(joy_codriver.GetRawButtonPressed(co_cargoIOToggle)) {
    //invert the cargo motor inversion.
    vic_cargoIO.SetInverted(!vic_cargoIO.GetInverted());
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

//Use these functions to test controls BEFORE putting them in periodic classes.
void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return StartRobot<Robot>(); }
#endif
