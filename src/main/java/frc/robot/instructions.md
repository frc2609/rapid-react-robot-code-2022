# Robot Code

## Imports
- `com.revrobotics.CANSparkMax` // motor controllers
- `com.revrobotics.CANSparkMaxLowLevel.MotorType` // to tell motor controller to use brushless motor
- `com.kauailabs.navx.frc.AHRS` // gyro
- `edu.wpi.first.wpilibj.SPI` // SPI port (gyro)
- `edu.wpi.first.wpilibj.XboxController` // Xbox Controller

## Online Install Links
https://software-metadata.revrobotics.com/REVLib.json
https://www.kauailabs.com/dist/frc/2022/navx_frc.json

## Variable Definitions
### Motors:
Front left (3):
```java
private final CANSparkMax frontLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
```

Rear left (4)
```java
private final CANSparkMax rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
```

Front Right (2)
```java
private final CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
```

Rear Right (1)
```java
private final CANSparkMax rearRightMotor = new CANSparkMax(1, MotorType.kBrushless);
```
### Joysticks and the Gyro:
NavX MXP is navx

Xbox Controller is joystick
```java
private final XboxController joystick = new XboxController(0);
private AHRS navx;
// below in robotInit():
try { navx = new AHRS(SPI.Port.kMXP); }
catch (RuntimeException ex) 
{ 
    System.out.println("Failed to initialize NAVX "); 
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
`hookMotor.getEncoder();`

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