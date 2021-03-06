#include "Robot.h"

#include <iostream>
#include <thread>

#include <cameraserver/CameraServer.h>

#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <ctre/phoenix/motorcontrol/GroupMotorControllers.h>
#include <ctre/Phoenix.h>

#include <frc/AnalogInput.h>
#include <frc/Compressor.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SpeedControllerGroup.h>

using namespace frc;

// Variable declaration //

// DRIVER //

// Left motors
WPI_TalonSRX tln_leftFront { 1 };
WPI_TalonSRX tln_leftBack { 2 };
SpeedControllerGroup grp_left { tln_leftFront, tln_leftBack };

// Left wheel encoder
Encoder enc_leftWheel{ 0, 1 };

// Right motors
WPI_TalonSRX tln_rightFront { 3 };
WPI_TalonSRX tln_rightBack { 4 };
SpeedControllerGroup grp_right { tln_rightFront, tln_rightBack };

double wheelVelMult = 0.75;

// Right wheel encoder
Encoder enc_rightWheel { 2, 3 };

// Left + right wheel diff. drive
DifferentialDrive drv_wheels { grp_left, grp_right };

// Elevator solenoids
DoubleSolenoid sol_frontActuators { 2, 3 },
    sol_backActuators { 6, 7 };

// Actuator wheel
WPI_VictorSPX vic_actuatorRight { 7 };
WPI_VictorSPX vic_actuatorLeft { 8 };
SpeedControllerGroup grp_act { vic_actuatorLeft, vic_actuatorRight };

double actWheelVelMult = 0.75;

// Controller for main driver
Joystick joy_driver { 0 };
// Axis and button values on the controller
int drvr_leftStickY = 1, drvr_rightStickY = 3,
    drvr_actWheelForward = 1, drvr_actWheelBack = 2,
    drvr_actsDown = 3, drvr_frontActUp = 4,
    drvr_backActUp = 5;

// CO-DRIVER //

// Arm connected to Cargo intake box
WPI_TalonSRX tln_arm { 5 };
// Multiplier for speed of the arm
const double armVelMult = 0.75;

DigitalInput limiterSwitchTop{4}, limiterSwitchBottom { 5 };

//axel + wheels used for intake and output of cargo balls
WPI_VictorSPX vic_cargoIO { 7 };

//controller for co driver (Arm control)
Joystick joy_codriver { 1 };
//axis and button values on the controller
int co_stickY = 1, co_cargoPowerToggle = 1, co_cargoIOToggle = 2;

// MISCELLANEOUS //

// used for encoder distances
const double pi = 3.1415926535897;

Compressor compressor;

// End variable declaration //

void Robot::RobotInit()
{
  compressor.Stop();
  // We need to run our vision program in a separate thread. If not, our robot
  // will not run.
  std::thread visionThread(VisionThread);
  visionThread.detach();

  // Sets the number of the samples the encoder gets to
  // average when calculating period.
  enc_rightWheel.SetSamplesToAverage(5);

  // Used to calculate distance traveled in one rotation.
  // Distance in this case is calculated in inches per second.
  // Formula: 1.0 / 360.0 * 2.0 * pi * radius
  enc_rightWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);

  // Sets the minimum number of inches per second for
  // the robot to be considered to be moving.
  enc_rightWheel.SetMinRate(1.0);

  // Repeat the above process for the left wheel encoder.
  enc_leftWheel.SetSamplesToAverage(5);
  enc_leftWheel.SetDistancePerPulse(1.0 / 360.0 * 2.0 * pi * 3.0);
  enc_leftWheel.SetMinRate(1.0);

  // Set the talon arm to Brake Mode.
  tln_arm.SetNeutralMode(NeutralMode::Brake);

  // Set the left wheel inverted so it matches with the right one.
  grp_left.SetInverted(true);

  // Set the right back talon inverted so it doesn't move
  // in the opposite direction of the other one.
  tln_rightBack.SetInverted(true);
  
  SmartDashboard::SetDefaultNumber("Drivetrain Speed Multiplier: ", 0.75);
}

void Robot::RobotPeriodic()
{
  // Update the values on the Smart Dashboard.
  UpdateDashboardValues();
}

void Robot::AutonomousInit()
{
  Robot::TeleopInit();
}

void Robot::AutonomousPeriodic()
{
  Robot::TeleopPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  // DRIVER CONTROLS //
  drv_wheels.TankDrive(wheelVelMult * joy_driver.GetRawAxis(drvr_leftStickY), wheelVelMult * joy_driver.GetRawAxis(drvr_rightStickY));

  /*
  // The POV must be held to drive.
  // forward
  if (joy_driver.GetPOV(0))
  {
    grp_act.Set(actWheelVelMult);
  }
  // back
  if (joy_driver.GetPOV(180))
  {
    grp_act.Set(-actWheelVelMult);
  }

  // The POV is not pressed
  if (joy_driver.GetPOV(-1))
  {
    grp_act.Set(0.0);
  }

  // The solenoids must be in their reverse state
  // in order to attempt to put them forward.
  if(joy_driver.GetRawButtonPressed(drvr_actsDown) && 
     sol_backActuators.Get() == DoubleSolenoid::Value::kReverse && 
     sol_frontActuators.Get() == DoubleSolenoid::Value::kReverse) {

    // Put both actuators down.
    sol_frontActuators.Set(DoubleSolenoid::Value::kForward);
    sol_backActuators.Set(DoubleSolenoid::Value::kForward);
  }

  // The back actuator must be down
  if(joy_driver.GetRawButtonPressed(drvr_backActUp) && 
     sol_backActuators.Get() == DoubleSolenoid::Value::kForward) {

    // Make the back actuators go up
    sol_backActuators.Set(DoubleSolenoid::Value::kReverse);
  }

  // The front actuator must be down
  if(joy_driver.GetRawButtonPressed(drvr_frontActUp) && 
     sol_frontActuators.Get() == DoubleSolenoid::Value::kForward) {

    // Make the front actuators go up
    sol_frontActuators.Set(DoubleSolenoid::Value::kReverse);
  }
  */

  // CO-DRIVER CONTROLS //

  // Neither limiter switch is hit, or
  // the bottom switch is hit and the arm is moving up, or
  // the top switch is hit and the arm is moving down
  if ((!limiterSwitchBottom.Get() && !limiterSwitchTop.Get()) ||
      (limiterSwitchBottom.Get() && joy_codriver.GetRawAxis(co_stickY) > 0) ||
      (limiterSwitchTop.Get() && joy_codriver.GetRawAxis(co_stickY) < 0))
  {
    // Move the arm using the joystick
    tln_arm.Set(joy_codriver.GetRawAxis(co_stickY) * armVelMult);
  }

  if (joy_codriver.GetRawButtonPressed(co_cargoPowerToggle))
  {
    // Set the cargo motor on
    vic_cargoIO.Set(0.5);
  }

  if (joy_codriver.GetRawButtonReleased(co_cargoPowerToggle))
  {
    // Set the cargo motor off
    vic_cargoIO.Set(0.0);
  }

  if (joy_codriver.GetRawButtonPressed(co_cargoIOToggle))
  {
    // Invert the direction of the cargo motor.
    vic_cargoIO.SetInverted(!vic_cargoIO.GetInverted());
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// Use these functions to test controls BEFORE putting them in periodic classes.
void Robot::TestInit() {}

void Robot::TestPeriodic() {}

// User-Defined Functions
void Robot::UpdateDashboardValues()
{
  SmartDashboard::PutBoolean("Cargo on: ", vic_cargoIO.Get() > 0.0);
  SmartDashboard::PutString("Cargo Intake Direction: ", (vic_cargoIO.GetInverted() ? "Out" : "In"));

  SmartDashboard::PutBoolean("Bottom pressed: ", limiterSwitchBottom.Get());
  SmartDashboard::PutBoolean("Top pressed: ", limiterSwitchTop.Get());

  SmartDashboard::PutNumber("Drivetrain Speed Multiplier: ", wheelVelMult);

  /*
  SmartDashboard::PutString("Front solenoid value: ",
                            (sol_frontActuators.Get() == DoubleSolenoid::kForward ? "Forward" : sol_frontActuators.Get() == DoubleSolenoid::kReverse ? "Reverse" : "Off"));
  SmartDashboard::PutString("Back solenoid value: ",
                            (sol_backActuators.Get() == DoubleSolenoid::kForward ? "Forward" : sol_backActuators.Get() == DoubleSolenoid::kReverse ? "Reverse" : "Off"));
                            */
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return StartRobot<Robot>();
}
#endif
