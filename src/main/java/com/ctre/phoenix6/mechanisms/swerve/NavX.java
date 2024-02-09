/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.mechanisms.swerve;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;

/**
 * Class description for the Pigeon 2 IMU sensor that measures orientation.
 * 
 * <pre>
 * // Constants used in Pigeon2 construction
 * final int kPigeon2Id = 0;
 * final String kPigeon2CANbus = "canivore";
 * 
 * // Construct the Pigeon 2
 * Pigeon2 pigeon = new Pigeon2(kPigeon2Id, kPigeon2CANbus);
 * 
 * // Configure the Pigeon2 for basic use
 * Pigeon2Configuration configs = new Pigeon2Configuration();
 * // This Pigeon is mounted X-up, so we should mount-pose with Pitch at 90 degrees
 * configs.MountPose.MountPoseYaw = 0;
 * configs.MountPose.MountPosePitch = 90;
 * configs.MountPose.MountPoseRoll = 0;
 * // This Pigeon has no need to trim the gyro
 * configs.GyroTrim.GyroScalarX = 0;
 * configs.GyroTrim.GyroScalarY = 0;
 * configs.GyroTrim.GyroScalarZ = 0;
 * // We want the thermal comp and no-motion cal enabled, with the compass disabled for best behavior
 * configs.Pigeon2Features.DisableNoMotionCalibration = false;
 * configs.Pigeon2Features.DisableTemperatureCompensation = false;
 * configs.Pigeon2Features.EnableCompass = false;
 * 
 * // Write these configs to the Pigeon2
 * pigeon.getConfigurator().apply(configs);
 * 
 * // Set the yaw to 0 degrees for initial use
 * pigeon.setYaw(0);
 * 
 * // Get Yaw Pitch Roll StatusSignals
 * var yaw = pigeon.getYaw();
 * var pitch = pigeon.getPitch();
 * var roll = pigeon.getRoll();
 * 
 * // Refresh and print these values
 * System.out.println("Yaw is " + yaw.refresh().toString());
 * System.out.println("Pitch is " + pitch.refresh().toString());
 * System.out.println("Roll is " + roll.refresh().toString());
 * </pre>
 */
public class NavX extends ParentDevice
{
    private Pigeon2Configurator _configurator;

    

    /**
     * Constructs a new Pigeon 2 sensor object.
     * <p>
     * Constructs the device using the default CAN bus for the system:
     * <ul>
     *   <li>"rio" on roboRIO
     *   <li>"can0" on Linux
     *   <li>"*" on Windows
     * </ul>
     *
     * @param deviceId    ID of the device, as configured in Phoenix Tuner.
     */
    public NavX(int deviceId)
    {
        this(deviceId, "");
    }
    /**
     * Constructs a new Pigeon 2 sensor object.
     *
     * @param deviceId    ID of the device, as configured in Phoenix Tuner.
     * @param canbus      Name of the CAN bus this device is on. Possible CAN bus strings are:
     *                    <ul>
     *                      <li>"rio" for the native roboRIO CAN bus
     *                      <li>CANivore name or serial number
     *                      <li>SocketCAN interface (non-FRC Linux only)
     *                      <li>"*" for any CANivore seen by the program
     *                      <li>empty string (default) to select the default for the system:
     *                      <ul>
     *                        <li>"rio" on roboRIO
     *                        <li>"can0" on Linux
     *                        <li>"*" on Windows
     *                      </ul>
     *                    </ul>
     */
    public NavX(int deviceId, String canbus)
    {
        super(deviceId, "pigeon 2", canbus);
        _configurator = new Pigeon2Configurator(this.deviceIdentifier);
        PlatformJNI.JNI_SimCreate(DeviceType.PRO_Pigeon2Type.value, deviceId);
    }

    /**
     * Gets the configurator to use with this device's configs
     *
     * @return Configurator for this object
     */
    public Pigeon2Configurator getConfigurator()
    {
        return this._configurator;
    }


    // private Pigeon2SimState _simState = null;
    /**
     * Get the simulation state for this device.
     * <p>
     * This function reuses an allocated simulation state
     * object, so it is safe to call this function multiple
     * times in a robot loop.
     *
     * @return Simulation state
     */
    // public Pigeon2SimState getSimState() {
    //     if (_simState == null)
    //         _simState = new Pigeon2SimState(this);
    //     return _simState;
    // }


        
    /**
     * App Major Version number.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return VersionMajor Status Signal Object
     */
    public StatusSignal<Integer> getVersionMajor()
    {
        return super.lookupStatusSignal(SpnValue.Version_Major.value, Integer.class, "VersionMajor", false);
    }
        
    /**
     * App Minor Version number.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return VersionMinor Status Signal Object
     */
    public StatusSignal<Integer> getVersionMinor()
    {
        return super.lookupStatusSignal(SpnValue.Version_Minor.value, Integer.class, "VersionMinor", false);
    }
        
    /**
     * App Bugfix Version number.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return VersionBugfix Status Signal Object
     */
    public StatusSignal<Integer> getVersionBugfix()
    {
        return super.lookupStatusSignal(SpnValue.Version_Bugfix.value, Integer.class, "VersionBugfix", false);
    }
        
    /**
     * App Build Version number.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return VersionBuild Status Signal Object
     */
    public StatusSignal<Integer> getVersionBuild()
    {
        return super.lookupStatusSignal(SpnValue.Version_Build.value, Integer.class, "VersionBuild", false);
    }
        
    /**
     * Full Version.  The format is a four byte value.
     * <p>
     * Full Version of firmware in device. The format is a four byte
     * value.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Version Status Signal Object
     */
    public StatusSignal<Integer> getVersion()
    {
        return super.lookupStatusSignal(SpnValue.Version_Full.value, Integer.class, "Version", false);
    }
        
    /**
     * Integer representing all faults
     * <p>
     * This returns the fault flags reported by the device. These are
     * device specific and are not used directly in typical applications.
     * Use the signal specific GetFault_*() methods instead.  
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 16777215
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return FaultField Status Signal Object
     */
    public StatusSignal<Integer> getFaultField()
    {
        return super.lookupStatusSignal(SpnValue.AllFaults.value, Integer.class, "FaultField", true);
    }
        
    /**
     * Integer representing all sticky faults
     * <p>
     * This returns the persistent "sticky" fault flags reported by the
     * device. These are device specific and are not used directly in
     * typical applications. Use the signal specific GetStickyFault_*()
     * methods instead.  
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 16777215
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFaultField Status Signal Object
     */
    public StatusSignal<Integer> getStickyFaultField()
    {
        return super.lookupStatusSignal(SpnValue.AllStickyFaults.value, Integer.class, "StickyFaultField", true);
    }
        
    /**
     * Current reported yaw of the Pigeon2.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -368640.0
     *   <li> <b>Maximum Value:</b> 368639.99725341797
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return Yaw Status Signal Object
     */
    public StatusSignal<Double> getYaw()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2Yaw.value, Double.class, "Yaw", true);
    }
        
    /**
     * Current reported pitch of the Pigeon2.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -90.0
     *   <li> <b>Maximum Value:</b> 89.9560546875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return Pitch Status Signal Object
     */
    public StatusSignal<Double> getPitch()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2Pitch.value, Double.class, "Pitch", true);
    }
        
    /**
     * Current reported roll of the Pigeon2.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -180.0
     *   <li> <b>Maximum Value:</b> 179.9560546875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return Roll Status Signal Object
     */
    public StatusSignal<Double> getRoll()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2Roll.value, Double.class, "Roll", true);
    }
        
    /**
     * The W component of the reported Quaternion.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.0001220852154804
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return QuatW Status Signal Object
     */
    public StatusSignal<Double> getQuatW()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2QuatW.value, Double.class, "QuatW", true);
    }
        
    /**
     * The X component of the reported Quaternion.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.0001220852154804
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return QuatX Status Signal Object
     */
    public StatusSignal<Double> getQuatX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2QuatX.value, Double.class, "QuatX", true);
    }
        
    /**
     * The Y component of the reported Quaternion.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.0001220852154804
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return QuatY Status Signal Object
     */
    public StatusSignal<Double> getQuatY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2QuatY.value, Double.class, "QuatY", true);
    }
        
    /**
     * The Z component of the reported Quaternion.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.0001220852154804
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return QuatZ Status Signal Object
     */
    public StatusSignal<Double> getQuatZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2QuatZ.value, Double.class, "QuatZ", true);
    }
        
    /**
     * The X component of the gravity vector.
     * <p>
     * This is the X component of the reported gravity-vector. The gravity
     * vector is not the acceleration experienced by the Pigeon2, rather
     * it is where the Pigeon2 believes "Down" is. This can be used for
     * mechanisms that are linearly related to gravity, such as an arm
     * pivoting about a point, as the contribution of gravity to the arm
     * is directly proportional to the contribution of gravity about one
     * of these primary axis.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.000030518509476
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return GravityVectorX Status Signal Object
     */
    public StatusSignal<Double> getGravityVectorX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2GravityVectorX.value, Double.class, "GravityVectorX", true);
    }
        
    /**
     * The Y component of the gravity vector.
     * <p>
     * This is the X component of the reported gravity-vector. The gravity
     * vector is not the acceleration experienced by the Pigeon2, rather
     * it is where the Pigeon2 believes "Down" is. This can be used for
     * mechanisms that are linearly related to gravity, such as an arm
     * pivoting about a point, as the contribution of gravity to the arm
     * is directly proportional to the contribution of gravity about one
     * of these primary axis.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.000030518509476
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return GravityVectorY Status Signal Object
     */
    public StatusSignal<Double> getGravityVectorY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2GravityVectorY.value, Double.class, "GravityVectorY", true);
    }
        
    /**
     * The Z component of the gravity vector.
     * <p>
     * This is the Z component of the reported gravity-vector. The gravity
     * vector is not the acceleration experienced by the Pigeon2, rather
     * it is where the Pigeon2 believes "Down" is. This can be used for
     * mechanisms that are linearly related to gravity, such as an arm
     * pivoting about a point, as the contribution of gravity to the arm
     * is directly proportional to the contribution of gravity about one
     * of these primary axis.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1.000030518509476
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return GravityVectorZ Status Signal Object
     */
    public StatusSignal<Double> getGravityVectorZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2GravityVectorZ.value, Double.class, "GravityVectorZ", true);
    }
        
    /**
     * Temperature of the Pigeon 2.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -128.0
     *   <li> <b>Maximum Value:</b> 127.99609375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return Temperature Status Signal Object
     */
    public StatusSignal<Double> getTemperature()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2Temperature.value, Double.class, "Temperature", true);
    }
        
    /**
     * Whether the no-motion calibration feature is enabled.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> 0
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return NoMotionEnabled Status Signal Object
     */
    public StatusSignal<Boolean> getNoMotionEnabled()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2NoMotionCalEnabled.value, Boolean.class, "NoMotionEnabled", true);
    }
        
    /**
     * The number of times a no-motion event occurred, wraps at 15.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 15
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return NoMotionCount Status Signal Object
     */
    public StatusSignal<Double> getNoMotionCount()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2NoMotionCount.value, Double.class, "NoMotionCount", true);
    }
        
    /**
     * Whether the temperature-compensation feature is disabled.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> 0
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return TemperatureCompensationDisabled Status Signal Object
     */
    public StatusSignal<Boolean> getTemperatureCompensationDisabled()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2TempCompDisabled.value, Boolean.class, "TemperatureCompensationDisabled", true);
    }
        
    /**
     * How long the Pigeon 2's been up in seconds, caps at 255 seconds.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> s
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return UpTime Status Signal Object
     */
    public StatusSignal<Double> getUpTime()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2UpTime.value, Double.class, "UpTime", true);
    }
        
    /**
     * The accumulated gyro about the X axis without any sensor fusing.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -23040.0
     *   <li> <b>Maximum Value:</b> 23039.9560546875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccumGyroX Status Signal Object
     */
    public StatusSignal<Double> getAccumGyroX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccumGyroX.value, Double.class, "AccumGyroX", true);
    }
        
    /**
     * The accumulated gyro about the Y axis without any sensor fusing.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -23040.0
     *   <li> <b>Maximum Value:</b> 23039.9560546875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccumGyroY Status Signal Object
     */
    public StatusSignal<Double> getAccumGyroY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccumGyroY.value, Double.class, "AccumGyroY", true);
    }
        
    /**
     * The accumulated gyro about the Z axis without any sensor fusing.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -23040.0
     *   <li> <b>Maximum Value:</b> 23039.9560546875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> deg
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccumGyroZ Status Signal Object
     */
    public StatusSignal<Double> getAccumGyroZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccumGyroZ.value, Double.class, "AccumGyroZ", true);
    }
        
    /**
     * Angular Velocity world X
     * <p>
     * This is the X component of the angular velocity with respect to the
     * world frame and is mount-calibrated.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2048.0
     *   <li> <b>Maximum Value:</b> 2047.99609375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AngularVelocityXWorld Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityXWorld()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityXWorld.value, Double.class, "AngularVelocityXWorld", true);
    }
        
    /**
     * Angular Velocity Quaternion Y Component
     * <p>
     * This is the Y component of the angular velocity with respect to the
     * world frame and is mount-calibrated.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2048.0
     *   <li> <b>Maximum Value:</b> 2047.99609375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AngularVelocityYWorld Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityYWorld()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityYWorld.value, Double.class, "AngularVelocityYWorld", true);
    }
        
    /**
     * Angular Velocity Quaternion Z Component
     * <p>
     * This is the Z component of the angular velocity with respect to the
     * world frame and is mount-calibrated.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2048.0
     *   <li> <b>Maximum Value:</b> 2047.99609375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AngularVelocityZWorld Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityZWorld()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityZWorld.value, Double.class, "AngularVelocityZWorld", true);
    }
        
    /**
     * The acceleration measured by Pigeon2 in the X direction.
     * <p>
     * This value includes the acceleration due to gravity. If this is
     * undesirable, get the gravity vector and subtract out the
     * contribution in this direction.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2.0
     *   <li> <b>Maximum Value:</b> 1.99993896484375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> g
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccelerationX Status Signal Object
     */
    public StatusSignal<Double> getAccelerationX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccelerationX.value, Double.class, "AccelerationX", true);
    }
        
    /**
     * The acceleration measured by Pigeon2 in the Y direction.
     * <p>
     * This value includes the acceleration due to gravity. If this is
     * undesirable, get the gravity vector and subtract out the
     * contribution in this direction.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2.0
     *   <li> <b>Maximum Value:</b> 1.99993896484375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> g
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccelerationY Status Signal Object
     */
    public StatusSignal<Double> getAccelerationY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccelerationY.value, Double.class, "AccelerationY", true);
    }
        
    /**
     * The acceleration measured by Pigeon2 in the Z direction.
     * <p>
     * This value includes the acceleration due to gravity. If this is
     * undesirable, get the gravity vector and subtract out the
     * contribution in this direction.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -2.0
     *   <li> <b>Maximum Value:</b> 1.99993896484375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> g
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 10.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return AccelerationZ Status Signal Object
     */
    public StatusSignal<Double> getAccelerationZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AccelerationZ.value, Double.class, "AccelerationZ", true);
    }
        
    /**
     * Measured supply voltage to the Pigeon2.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 31.99951171875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> V
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     *   </ul>
     * 
     * @return SupplyVoltage Status Signal Object
     */
    public StatusSignal<Double> getSupplyVoltage()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2_SupplyVoltage.value, Double.class, "SupplyVoltage", true);
    }
        
    /**
     * The angular velocity (ω) of the Pigeon 2 about the device's X axis.
     * <p>
     * This value is not mount-calibrated
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1998.048780487805
     *   <li> <b>Maximum Value:</b> 1997.987804878049
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return AngularVelocityXDevice Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityXDevice()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityX.value, Double.class, "AngularVelocityXDevice", true);
    }
        
    /**
     * The angular velocity (ω) of the Pigeon 2 about the device's Y axis.
     * <p>
     * This value is not mount-calibrated
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1998.048780487805
     *   <li> <b>Maximum Value:</b> 1997.987804878049
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return AngularVelocityYDevice Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityYDevice()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityY.value, Double.class, "AngularVelocityYDevice", true);
    }
        
    /**
     * The angular velocity (ω) of the Pigeon 2 about the device's Z axis.
     * <p>
     * This value is not mount-calibrated
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -1998.048780487805
     *   <li> <b>Maximum Value:</b> 1997.987804878049
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> dps
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return AngularVelocityZDevice Status Signal Object
     */
    public StatusSignal<Double> getAngularVelocityZDevice()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2AngularVelocityZ.value, Double.class, "AngularVelocityZDevice", true);
    }
        
    /**
     * The biased magnitude of the magnetic field measured by the Pigeon 2
     * in the X direction. This is only valid after performing a
     * magnetometer calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return MagneticFieldX Status Signal Object
     */
    public StatusSignal<Double> getMagneticFieldX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2MagneticFieldX.value, Double.class, "MagneticFieldX", true);
    }
        
    /**
     * The biased magnitude of the magnetic field measured by the Pigeon 2
     * in the Y direction. This is only valid after performing a
     * magnetometer calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return MagneticFieldY Status Signal Object
     */
    public StatusSignal<Double> getMagneticFieldY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2MagneticFieldY.value, Double.class, "MagneticFieldY", true);
    }
        
    /**
     * The biased magnitude of the magnetic field measured by the Pigeon 2
     * in the Z direction. This is only valid after performing a
     * magnetometer calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return MagneticFieldZ Status Signal Object
     */
    public StatusSignal<Double> getMagneticFieldZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2MagneticFieldZ.value, Double.class, "MagneticFieldZ", true);
    }
        
    /**
     * The raw magnitude of the magnetic field measured by the Pigeon 2 in
     * the X direction. This is only valid after performing a magnetometer
     * calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return RawMagneticFieldX Status Signal Object
     */
    public StatusSignal<Double> getRawMagneticFieldX()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2RawMagneticFieldX.value, Double.class, "RawMagneticFieldX", true);
    }
        
    /**
     * The raw magnitude of the magnetic field measured by the Pigeon 2 in
     * the Y direction. This is only valid after performing a magnetometer
     * calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return RawMagneticFieldY Status Signal Object
     */
    public StatusSignal<Double> getRawMagneticFieldY()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2RawMagneticFieldY.value, Double.class, "RawMagneticFieldY", true);
    }
        
    /**
     * The raw magnitude of the magnetic field measured by the Pigeon 2 in
     * the Z direction. This is only valid after performing a magnetometer
     * calibration.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -19660.8
     *   <li> <b>Maximum Value:</b> 19660.2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> uT
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return RawMagneticFieldZ Status Signal Object
     */
    public StatusSignal<Double> getRawMagneticFieldZ()
    {
        return super.lookupStatusSignal(SpnValue.Pigeon2RawMagneticFieldZ.value, Double.class, "RawMagneticFieldZ", true);
    }
        
    /**
     * Whether the device is Phoenix Pro licensed.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return IsProLicensed Status Signal Object
     */
    public StatusSignal<Boolean> getIsProLicensed()
    {
        return super.lookupStatusSignal(SpnValue.Version_IsProLicensed.value, Boolean.class, "IsProLicensed", true);
    }
        
    /**
     * Hardware fault occurred
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_Hardware Status Signal Object
     */
    public StatusSignal<Boolean> getFault_Hardware()
    {
        return super.lookupStatusSignal(SpnValue.Fault_Hardware.value, Boolean.class, "Fault_Hardware", true);
    }
        
    /**
     * Hardware fault occurred
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_Hardware Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_Hardware()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_Hardware.value, Boolean.class, "StickyFault_Hardware", true);
    }
        
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_Undervoltage Status Signal Object
     */
    public StatusSignal<Boolean> getFault_Undervoltage()
    {
        return super.lookupStatusSignal(SpnValue.Fault_Undervoltage.value, Boolean.class, "Fault_Undervoltage", true);
    }
        
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_Undervoltage Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_Undervoltage()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_Undervoltage.value, Boolean.class, "StickyFault_Undervoltage", true);
    }
        
    /**
     * Device boot while detecting the enable signal
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_BootDuringEnable Status Signal Object
     */
    public StatusSignal<Boolean> getFault_BootDuringEnable()
    {
        return super.lookupStatusSignal(SpnValue.Fault_BootDuringEnable.value, Boolean.class, "Fault_BootDuringEnable", true);
    }
        
    /**
     * Device boot while detecting the enable signal
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_BootDuringEnable Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_BootDuringEnable()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_BootDuringEnable.value, Boolean.class, "StickyFault_BootDuringEnable", true);
    }
        
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_UnlicensedFeatureInUse Status Signal Object
     */
    public StatusSignal<Boolean> getFault_UnlicensedFeatureInUse()
    {
        return super.lookupStatusSignal(SpnValue.Fault_UnlicensedFeatureInUse.value, Boolean.class, "Fault_UnlicensedFeatureInUse", true);
    }
        
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_UnlicensedFeatureInUse Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_UnlicensedFeatureInUse.value, Boolean.class, "StickyFault_UnlicensedFeatureInUse", true);
    }
        
    /**
     * Bootup checks failed: Accelerometer
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_BootupAccelerometer Status Signal Object
     */
    public StatusSignal<Boolean> getFault_BootupAccelerometer()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_BootupAccel.value, Boolean.class, "Fault_BootupAccelerometer", true);
    }
        
    /**
     * Bootup checks failed: Accelerometer
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_BootupAccelerometer Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_BootupAccelerometer()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_BootupAccel.value, Boolean.class, "StickyFault_BootupAccelerometer", true);
    }
        
    /**
     * Bootup checks failed: Gyroscope
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_BootupGyroscope Status Signal Object
     */
    public StatusSignal<Boolean> getFault_BootupGyroscope()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_BootupGyros.value, Boolean.class, "Fault_BootupGyroscope", true);
    }
        
    /**
     * Bootup checks failed: Gyroscope
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_BootupGyroscope Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_BootupGyroscope()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_BootupGyros.value, Boolean.class, "StickyFault_BootupGyroscope", true);
    }
        
    /**
     * Bootup checks failed: Magnetometer
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_BootupMagnetometer Status Signal Object
     */
    public StatusSignal<Boolean> getFault_BootupMagnetometer()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_BootupMagne.value, Boolean.class, "Fault_BootupMagnetometer", true);
    }
        
    /**
     * Bootup checks failed: Magnetometer
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_BootupMagnetometer Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_BootupMagnetometer()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_BootupMagne.value, Boolean.class, "StickyFault_BootupMagnetometer", true);
    }
        
    /**
     * Motion Detected during bootup.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_BootIntoMotion Status Signal Object
     */
    public StatusSignal<Boolean> getFault_BootIntoMotion()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_BootIntoMotion.value, Boolean.class, "Fault_BootIntoMotion", true);
    }
        
    /**
     * Motion Detected during bootup.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_BootIntoMotion Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_BootIntoMotion()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_BootIntoMotion.value, Boolean.class, "StickyFault_BootIntoMotion", true);
    }
        
    /**
     * Motion stack data acquisition was slower than expected
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_DataAcquiredLate Status Signal Object
     */
    public StatusSignal<Boolean> getFault_DataAcquiredLate()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_DataAcquiredLate.value, Boolean.class, "Fault_DataAcquiredLate", true);
    }
        
    /**
     * Motion stack data acquisition was slower than expected
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_DataAcquiredLate Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_DataAcquiredLate()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_DataAcquiredLate.value, Boolean.class, "StickyFault_DataAcquiredLate", true);
    }
        
    /**
     * Motion stack loop time was slower than expected.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_LoopTimeSlow Status Signal Object
     */
    public StatusSignal<Boolean> getFault_LoopTimeSlow()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_LoopTimeSlow.value, Boolean.class, "Fault_LoopTimeSlow", true);
    }
        
    /**
     * Motion stack loop time was slower than expected.
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_LoopTimeSlow Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_LoopTimeSlow()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_LoopTimeSlow.value, Boolean.class, "StickyFault_LoopTimeSlow", true);
    }
        
    /**
     * Magnetometer values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_SaturatedMagnetometer Status Signal Object
     */
    public StatusSignal<Boolean> getFault_SaturatedMagnetometer()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_SaturatedMagne.value, Boolean.class, "Fault_SaturatedMagnetometer", true);
    }
        
    /**
     * Magnetometer values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_SaturatedMagnetometer Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_SaturatedMagnetometer()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_SaturatedMagne.value, Boolean.class, "StickyFault_SaturatedMagnetometer", true);
    }
        
    /**
     * Accelerometer values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_SaturatedAccelerometer Status Signal Object
     */
    public StatusSignal<Boolean> getFault_SaturatedAccelerometer()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_SaturatedAccel.value, Boolean.class, "Fault_SaturatedAccelerometer", true);
    }
        
    /**
     * Accelerometer values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_SaturatedAccelerometer Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_SaturatedAccelerometer()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_SaturatedAccel.value, Boolean.class, "StickyFault_SaturatedAccelerometer", true);
    }
        
    /**
     * Gyroscope values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return Fault_SaturatedGyroscope Status Signal Object
     */
    public StatusSignal<Boolean> getFault_SaturatedGyroscope()
    {
        return super.lookupStatusSignal(SpnValue.Fault_PIGEON2_SaturatedGyros.value, Boolean.class, "Fault_SaturatedGyroscope", true);
    }
        
    /**
     * Gyroscope values are saturated
     * 
     *   <ul>
     *   <li> <b>Default Value:</b> False
     *   </ul>
     * 
     * Default Rates:
     *   <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     *   </ul>
     * 
     * @return StickyFault_SaturatedGyroscope Status Signal Object
     */
    public StatusSignal<Boolean> getStickyFault_SaturatedGyroscope()
    {
        return super.lookupStatusSignal(SpnValue.StickyFault_PIGEON2_SaturatedGyros.value, Boolean.class, "StickyFault_SaturatedGyroscope", true);
    }

    

    /**
     * Control motor with generic control request object.
     * <p>
     * User must make sure the specified object is castable to a valid control request,
     * otherwise this function will fail at run-time and return the NotSupported StatusCode
     *
     * @param request                Control object to request of the device
     * @return Status Code of the request, 0 is OK
     */
    public StatusCode setControl(ControlRequest request)
    {
        
        return StatusCode.NotSupported;
    }

    
    /**
     * The yaw to set the Pigeon2 to right now.
     * 
     * @param newValue Value to set to. Units are in deg.
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode setYaw(double newValue, double timeoutSeconds) {
        return getConfigurator().setYaw(newValue, timeoutSeconds);
    }
    /**
     * The yaw to set the Pigeon2 to right now.
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @param newValue Value to set to. Units are in deg.
     * @return StatusCode of the set command
     */
    public StatusCode setYaw(double newValue) {
        return setYaw(newValue, 0.050);
    }
    
    /**
     * Clear the sticky faults in the device.
     * <p>
     * This typically has no impact on the device functionality.  Instead,
     * it just clears telemetry faults that are accessible via API and
     * Tuner Self-Test.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFaults(double timeoutSeconds) {
        return getConfigurator().clearStickyFaults(timeoutSeconds);
    }
    /**
     * Clear the sticky faults in the device.
     * <p>
     * This typically has no impact on the device functionality.  Instead,
     * it just clears telemetry faults that are accessible via API and
     * Tuner Self-Test.
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFaults() {
        return clearStickyFaults(0.050);
    }
    
    /**
     * Clear sticky fault: Hardware fault occurred
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_Hardware(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_Hardware(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Hardware fault occurred
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_Hardware() {
        return clearStickyFault_Hardware(0.050);
    }
    
    /**
     * Clear sticky fault: Device supply voltage dropped to near brownout
     * levels
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_Undervoltage(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_Undervoltage(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Device supply voltage dropped to near brownout
     * levels
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_Undervoltage() {
        return clearStickyFault_Undervoltage(0.050);
    }
    
    /**
     * Clear sticky fault: Device boot while detecting the enable signal
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootDuringEnable(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootDuringEnable(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Device boot while detecting the enable signal
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootDuringEnable() {
        return clearStickyFault_BootDuringEnable(0.050);
    }
    
    /**
     * Clear sticky fault: Bootup checks failed: Accelerometer
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupAccelerometer(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootupAccelerometer(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Bootup checks failed: Accelerometer
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupAccelerometer() {
        return clearStickyFault_BootupAccelerometer(0.050);
    }
    
    /**
     * Clear sticky fault: Bootup checks failed: Gyroscope
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupGyroscope(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootupGyroscope(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Bootup checks failed: Gyroscope
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupGyroscope() {
        return clearStickyFault_BootupGyroscope(0.050);
    }
    
    /**
     * Clear sticky fault: Bootup checks failed: Magnetometer
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupMagnetometer(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootupMagnetometer(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Bootup checks failed: Magnetometer
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootupMagnetometer() {
        return clearStickyFault_BootupMagnetometer(0.050);
    }
    
    /**
     * Clear sticky fault: Motion Detected during bootup.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootIntoMotion(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootIntoMotion(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Motion Detected during bootup.
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_BootIntoMotion() {
        return clearStickyFault_BootIntoMotion(0.050);
    }
    
    /**
     * Clear sticky fault: Motion stack data acquisition was slower than
     * expected
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_DataAcquiredLate(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_DataAcquiredLate(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Motion stack data acquisition was slower than
     * expected
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_DataAcquiredLate() {
        return clearStickyFault_DataAcquiredLate(0.050);
    }
    
    /**
     * Clear sticky fault: Motion stack loop time was slower than
     * expected.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_LoopTimeSlow(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_LoopTimeSlow(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Motion stack loop time was slower than
     * expected.
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_LoopTimeSlow() {
        return clearStickyFault_LoopTimeSlow(0.050);
    }
    
    /**
     * Clear sticky fault: Magnetometer values are saturated
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedMagnetometer(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_SaturatedMagnetometer(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Magnetometer values are saturated
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedMagnetometer() {
        return clearStickyFault_SaturatedMagnetometer(0.050);
    }
    
    /**
     * Clear sticky fault: Accelerometer values are saturated
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedAccelerometer(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_SaturatedAccelerometer(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Accelerometer values are saturated
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedAccelerometer() {
        return clearStickyFault_SaturatedAccelerometer(0.050);
    }
    
    /**
     * Clear sticky fault: Gyroscope values are saturated
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedGyroscope(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_SaturatedGyroscope(timeoutSeconds);
    }
    /**
     * Clear sticky fault: Gyroscope values are saturated
     * <p>
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * @return StatusCode of the set command
     */
    public StatusCode clearStickyFault_SaturatedGyroscope() {
        return clearStickyFault_SaturatedGyroscope(0.050);
    }
}

