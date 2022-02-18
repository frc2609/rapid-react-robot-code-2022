# Robot Code

## Imports
- `com.revrobotics.CANSparkMax` // Spark Max motor controllers
- `com.revrobotics.CANSparkMaxLowLevel.MotorType` // to tell motor controller to use brushless motor
- `com.ctre.phoenix.motorcontrol.can.VictorSRX` // VictorSPX motor controllers
- `com.kauailabs.navx.frc.AHRS` // gyro
- `edu.wpi.first.wpilibj.SPI` // SPI port (gyro)
- `edu.wpi.first.wpilibj.Joystick` // Joystick
- `edu.wpi.first.wpilibj.XboxController` // Xbox Controller
- `frc.robot.Constants` // constants, e.g. LEFT_STICK_Y_AXIS
- `edu.wpi.first.wpilibj.SerialPort` // USB ports on RoboRIO

## Online Install Links
https://software-metadata.revrobotics.com/REVLib.json
https://www.kauailabs.com/dist/frc/2022/navx_frc.json
https://github.com/CrossTheRoadElec/Phoenix-Releases/releases <- install this to use the VictorSPX motor controllers

## Variable Definitions
### Motors:
Spark Max via CAN bus:
```java
private final CANSparkMax motorName = new CANSparkMax(Constants.*_MOTOR, MotorType.kBrushless);
```
Replace * with a motor name (e.g. HOOK)

Victor SPX via CAN bus:
```java
private final VictorSPX motorName = new VictorSPX(Constants.*_MOTOR);
```
Replace * with a motor name (e.g. SHOOTER_LEFT_MOTOR)

### Joysticks:
```java
private final Joystick joystickName = new Joystick(Constants.);
```

### NavX:
```java
private AHRS navx;
// below in robotInit():
try { navx = new AHRS(SPI.Port.kMXP); }
catch (RuntimeException ex) 
{ 
    System.out.println("Failed to initialize NAVX "); 
}
```

### NavX USB:
```java
private AHRS navxUSB;
// below in robotInit():
try { navx = new AHRS(SerialPort.Port.kUSB1); }
catch (RuntimeException ex) 
{ 
    System.out.println("Failed to initialize NAVX ON USB"); 
}
```

## Motor Control
### Calculating Motor Values:
X = get x axis to the power of 3: 

```java
double driveX = Math.pow(joystick.getRawAxis(0), 3);
```

Y = get y axis to the power of 3: 

```java
double driveY = Math.pow(joystick.getRawAxis(1), 3);
```

Left motors = y - x

```java
double leftMotors = driveY - driveX;
```

Right motors = y + x

```java
double rightMotors = driveY + driveX;
```

### Setting the motors:

Front left set = left motors

Rear left set = (negative) left motors

Front right set = (negative) right motors

Rear right set = right motors

```java
frontLeftMotor.set(leftMotors);
rearLeftMotor.set(-leftMotors);
frontRightMotor.set(-rightMotors);
rearRightMotor.set(rightMotors);
```
(Colours aren't there for some reason in the preview)

### Using Encoders:
`hookMotor.getEncoder().getPosition();`

## Displaying Information Using Smart Dashboard
Use this to display information on the Smart Dashboard console.

You probably want to put these in `robotPeriodic()`

`SmartDashboard.putTYPE("info about value", value)`

Replace TYPE with a type (e.g. Boolean or Number)

For example:

```java
SmartDashboard.putBoolean("NavX connection status:", navx.isConnected());
```

## NavX Methods

`navx.zeroYaw() // use to reset yaw which can drift`

`navx.getRoll()`

`navx.getPitch()`

`navx.getYaw()`

## Notes

Change 'P' of PID to change motor speed

`// TODO: ` at the start of a comment underline in it blue and show it in the "problems" section.