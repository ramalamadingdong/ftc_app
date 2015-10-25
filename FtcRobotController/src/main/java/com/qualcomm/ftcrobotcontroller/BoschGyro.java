package com.qualcomm.ftcrobotcontroller;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.locks.Lock;

/**
 * Created by Aluminati on 9/27/2015.
 * I2cPortReadyCallback extends HardwareDevice
 * Hardware device has 4 functions
 *    String getDeviceName();
 *    String getConnectionInfo();
 *    int getVersion();
 *    void close();
 */

public class BoschGyro implements HardwareDevice,I2cController.I2cPortReadyCallback {

    final static byte bCHIP_ID_VALUE = (byte) 0xa0;

    /**
     * Sensor modes are described in Table 3-5 (p21) of the BNO055 specification,
     * where they are termed "operation modes".
     */
    enum SENSOR_MODE {
        CONFIG(0X00),
        ACCONLY(0X01),
        MAGONLY(0X02),
        GYRONLY(0X03),
        ACCMAG(0X04),
        ACCGYRO(0X05),
        MAGGYRO(0X06),
        AMG(0X07),
        IMU(0X08),
        COMPASS(0X09),
        M4G(0X0A),
        NDOF_FMC_OFF(0X0B),
        NDOF(0X0C);
        //------------------------------------------------------------------------------------------
        public final byte bVal;

        SENSOR_MODE(int i) {
            this.bVal = (byte) i;
        }
    }

    /**
     * REGISTER provides symbolic names for each of the BNO055 device registers
     */
    enum REGISTER {
        /**
         * Controls which of the two register pages are visible
         */
        PAGE_ID(0X07),

        CHIP_ID(0x00),
        ACCEL_REV_ID(0x01),
        MAG_REV_ID(0x02),
        GYRO_REV_ID(0x03),
        SW_REV_ID_LSB(0x04),
        SW_REV_ID_MSB(0x05),
        BL_REV_ID(0X06),

        /**
         * Acceleration data register
         */
        ACCEL_DATA_X_LSB(0X08),
        ACCEL_DATA_X_MSB(0X09),
        ACCEL_DATA_Y_LSB(0X0A),
        ACCEL_DATA_Y_MSB(0X0B),
        ACCEL_DATA_Z_LSB(0X0C),
        ACCEL_DATA_Z_MSB(0X0D),

        /**
         * Magnetometer data register
         */
        MAG_DATA_X_LSB(0X0E),
        MAG_DATA_X_MSB(0X0F),
        MAG_DATA_Y_LSB(0X10),
        MAG_DATA_Y_MSB(0X11),
        MAG_DATA_Z_LSB(0X12),
        MAG_DATA_Z_MSB(0X13),

        /**
         * Gyro data registers
         */
        GYRO_DATA_X_LSB(0X14),
        GYRO_DATA_X_MSB(0X15),
        GYRO_DATA_Y_LSB(0X16),
        GYRO_DATA_Y_MSB(0X17),
        GYRO_DATA_Z_LSB(0X18),
        GYRO_DATA_Z_MSB(0X19),

        /**
         * Euler data registers
         */
        EULER_H_LSB(0X1A),
        EULER_H_MSB(0X1B),
        EULER_R_LSB(0X1C),
        EULER_R_MSB(0X1D),
        EULER_P_LSB(0X1E),
        EULER_P_MSB(0X1F),

        /**
         * Quaternion data registers
         */
        QUATERNION_DATA_W_LSB(0X20),
        QUATERNION_DATA_W_MSB(0X21),
        QUATERNION_DATA_X_LSB(0X22),
        QUATERNION_DATA_X_MSB(0X23),
        QUATERNION_DATA_Y_LSB(0X24),
        QUATERNION_DATA_Y_MSB(0X25),
        QUATERNION_DATA_Z_LSB(0X26),
        QUATERNION_DATA_Z_MSB(0X27),

        /**
         * Linear acceleration data registers
         */
        LINEAR_ACCEL_DATA_X_LSB(0X28),
        LINEAR_ACCEL_DATA_X_MSB(0X29),
        LINEAR_ACCEL_DATA_Y_LSB(0X2A),
        LINEAR_ACCEL_DATA_Y_MSB(0X2B),
        LINEAR_ACCEL_DATA_Z_LSB(0X2C),
        LINEAR_ACCEL_DATA_Z_MSB(0X2D),

        /**
         * Gravity data registers
         */
        GRAVITY_DATA_X_LSB(0X2E),
        GRAVITY_DATA_X_MSB(0X2F),
        GRAVITY_DATA_Y_LSB(0X30),
        GRAVITY_DATA_Y_MSB(0X31),
        GRAVITY_DATA_Z_LSB(0X32),
        GRAVITY_DATA_Z_MSB(0X33),

        /**
         * Temperature data register
         */
        TEMP(0X34),

        /**
         * Status registers
         */
        CALIB_STAT(0X35),
        SELFTEST_RESULT(0X36),
        INTR_STAT(0X37),

        SYS_CLK_STAT(0X38),
        SYS_STAT(0X39),
        SYS_ERR(0X3A),

        /**
         * Unit selection register
         */
        UNIT_SEL(0X3B),
        DATA_SELECT(0X3C),

        /**
         * Mode registers
         */
        OPR_MODE(0X3D),
        PWR_MODE(0X3E),

        SYS_TRIGGER(0X3F),
        TEMP_SOURCE(0X40),

        /**
         * Axis remap registers
         */
        AXIS_MAP_CONFIG(0X41),
        AXIS_MAP_SIGN(0X42),

        /**
         * SIC registers
         */
        SIC_MATRIX_0_LSB(0X43),
        SIC_MATRIX_0_MSB(0X44),
        SIC_MATRIX_1_LSB(0X45),
        SIC_MATRIX_1_MSB(0X46),
        SIC_MATRIX_2_LSB(0X47),
        SIC_MATRIX_2_MSB(0X48),
        SIC_MATRIX_3_LSB(0X49),
        SIC_MATRIX_3_MSB(0X4A),
        SIC_MATRIX_4_LSB(0X4B),
        SIC_MATRIX_4_MSB(0X4C),
        SIC_MATRIX_5_LSB(0X4D),
        SIC_MATRIX_5_MSB(0X4E),
        SIC_MATRIX_6_LSB(0X4F),
        SIC_MATRIX_6_MSB(0X50),
        SIC_MATRIX_7_LSB(0X51),
        SIC_MATRIX_7_MSB(0X52),
        SIC_MATRIX_8_LSB(0X53),
        SIC_MATRIX_8_MSB(0X54),

        /**
         * Accelerometer Offset registers
         */
        ACCEL_OFFSET_X_LSB(0X55),
        ACCEL_OFFSET_X_MSB(0X56),
        ACCEL_OFFSET_Y_LSB(0X57),
        ACCEL_OFFSET_Y_MSB(0X58),
        ACCEL_OFFSET_Z_LSB(0X59),
        ACCEL_OFFSET_Z_MSB(0X5A),

        /**
         * Magnetometer Offset registers
         */
        MAG_OFFSET_X_LSB(0X5B),
        MAG_OFFSET_X_MSB(0X5C),
        MAG_OFFSET_Y_LSB(0X5D),
        MAG_OFFSET_Y_MSB(0X5E),
        MAG_OFFSET_Z_LSB(0X5F),
        MAG_OFFSET_Z_MSB(0X60),

        /**
         * Gyroscope Offset register s
         */
        GYRO_OFFSET_X_LSB(0X61),
        GYRO_OFFSET_X_MSB(0X62),
        GYRO_OFFSET_Y_LSB(0X63),
        GYRO_OFFSET_Y_MSB(0X64),
        GYRO_OFFSET_Z_LSB(0X65),
        GYRO_OFFSET_Z_MSB(0X66),

        /**
         * Radius registers
         */
        ACCEL_RADIUS_LSB(0X67),
        ACCEL_RADIUS_MSB(0X68),
        MAG_RADIUS_LSB(0X69),
        MAG_RADIUS_MSB(0X6A);
        //------------------------------------------------------------------------------------------
        public final byte bVal;

        private REGISTER(int i) {
            this.bVal = (byte) i;
        }
    }

    enum TEMPUNIT   { CELSIUS(0), FARENHEIT(1);                            public final byte bVal; TEMPUNIT(int i)  { bVal =(byte)i; }}
    enum ANGLEUNIT  { DEGREES(0), RADIANS(1);                              public final byte bVal; ANGLEUNIT(int i) { bVal =(byte)i; }}
    enum ACCELUNIT  { METERS_PERSEC_PERSEC(0), MILLIGALS(1);               public final byte bVal; ACCELUNIT(int i) { bVal =(byte)i; }}
    enum PITCHMODE  { WINDOWS(0), ANDROID(1);                              public final byte bVal; PITCHMODE(int i) { bVal =(byte)i; }}

    private enum States {
        IMU_WAIT_FOR_COMPLETE,
        IMU_INIT,
        IMU_CHECK_ID,
        IMU_ID_VERIFY,
        IMU_RESET,
        IMU_RESET_VERIFY,
        IMU_RESET_VERIFY2,
        IMU_WAIT_FOR_SELF_TEST,
        IMU_SET_POWER_MODE,
        IMU_SET_PAGE_ID,
        IMU_SET_UNITS,
        IMU_MODE_TRANSITION,
        IMU_MODE_VERIFY,
        IMU_MODE_VERIFY2,
        IMU_VERIFY_CALIBRATION,
        IMU_READ_STATUS,
        IMU_READ_STATUS2,
        IMU_RUNNING,
        IMU_DELAY
    }

    private final I2cDevice imu;
    private ElapsedTime runtime;

    public volatile int I2C_ADDRESS = 0x28 * 2;
    public volatile boolean localReady = true;
    private final byte dataRegValue = (byte)0xFF;
    private final int i2cBufferSize = 26; //Size of any extra buffers that will hold any incoming or outgoing cache data
    private final byte[] readCache;
    private final Lock readCacheLock;
    private final byte[] writeCache; //This cache will hold the bytes which are to be written to the interface
    private final Lock writeCacheLock; //A lock on access to the IMU's I2C write cache
    private States imuState;
    private States nextImuState;
    private States nextImuStateStatus;
    private boolean initComplete;
    private byte[] outboundBytes;
    private double delayTime;
    private double relativeHeading;

    /** whether to use the external or internal 32.768khz crystal. External crystal
     * use is recommended by the BNO055 specification. */
    private boolean          useExternalCrystal  = true;

    /** units in which temperature are measured. See Section 3.6.1 (p31) of the BNO055 specification */
    private TEMPUNIT         temperatureUnit     = TEMPUNIT.CELSIUS;
    /** units in which angles and angular rates are measured. See Section 3.6.1 (p31) of the BNO055 specification */
    private ANGLEUNIT        angleunit           = ANGLEUNIT.DEGREES;
    /** units in which accelerations are measured. See Section 3.6.1 (p31) of the BNO055 specification */
    private ACCELUNIT        accelunit           = ACCELUNIT.METERS_PERSEC_PERSEC;
    /** directional convention for measureing pitch angles. See Section 3.6.1 (p31) of the BNO055 specification */
    private PITCHMODE        pitchmode           = PITCHMODE.ANDROID;    // Section 3.6.2

    public BoschGyro(HardwareMap currentHWmap,String configuredIMUname) {
        imu = currentHWmap.i2cDevice.get(configuredIMUname);

        this.outboundBytes = new byte[i2cBufferSize];

        this.readCache = this.imu.getI2cReadCache();
        this.readCacheLock = this.imu.getI2cReadCacheLock();
        this.writeCache = this.imu.getI2cWriteCache();
        this.writeCacheLock = this.imu.getI2cWriteCacheLock();

        this.imu.registerForI2cPortReadyCallback(this);

        this.imuState = States.IMU_INIT;
        this.nextImuState = States.IMU_INIT;
        this.nextImuStateStatus = States.IMU_INIT;
        this.initComplete = false;
        delayTime = 0;
        runtime = new ElapsedTime();
        relativeHeading = 0.0;
    }

    /**
     * This class is called during the "init_loop" of the Op Mode
     * It takes a couple seconds for the IMU to initiailize and calibrate
     * The initComplete is set true when calibration is done and the
     * IMU is ready for use
     * @return true is ready for use
     */
    public boolean initComplete() {
        return this.initComplete;
    }

    /**
     * Utility function to "dump" memory to the logcat
     * @param n - array of bytes to display the hex values
     * @return String that can be output to the log
     */
    public static String hex(byte n[]) {
        StringBuilder sb = new StringBuilder();
        for (byte b : n) {
            sb.append(String.format("%02X ", b));
        }
        return sb.toString();
    }

    /**
     * The IMU constantly runs in the background. The QualComm software continuously polls
     * all the sensors and stores the data in the cache. This function just reads what is
     * in the cache and returns the heading
     * @return heading in degrees range from 0 to 360
     */
    public double getHeading()
    {
        short headingInt;
        double h;

        this.readCacheLock.lock();

        headingInt = (short)(this.readCache[I2cController.I2C_BUFFER_START_ADDRESS]  +
                this.readCache[I2cController.I2C_BUFFER_START_ADDRESS+1] * 256);

        h = ((float)headingInt) * 1.0/16.0;

        this.readCacheLock.unlock();

        if (h < 0.0) {
            h = h + 360.0;
        }
        return h;
    }

    /**
     * This sets the "relative heading". Calling this routine will set the relative heading
     * to the current heading. the getRelativeHeading will return the number of degrees relative
     * to the starting point. Basically this sets what "0" is. Call this routine just before you
     * move and then you use to getRelativeHeading to see how many degrees you have turned from
     * the current position.
     */
    public void setZeroHeading()
    {
        relativeHeading = getHeading();
    }

    /**
     * Get what the zero heading is set to
     * @return
     */
    public double getZeroHeading()
    {
        return relativeHeading;
    }
    /**
     * getRelativeHeading computes the angle relative to when the setZeroHeading is called.
     * @return relative heading -180 to 180
     */
    public double getRelativeHeading() {
        double h;

        h = getHeading() - relativeHeading;
        if (h < -180.0) {
            h = h + 360;
        }
        else if (h > 180.0)
        {
            h = h -360;
        }
        return h;
    }
    /**
     * The following status information is only valid after the unit has been commanded to
     * read the status data from the BNO. Don't assume it is always valid unless the last
     * command issued was to read the status
     */

    /**
     * Calibration is a challenge for the Magnetometer and Accelerometer. The robot could be
     * pre-calibrated and stored in a file on the phone. This class currently does read or
     * write the calibration data to the phone.
     */

    /**
     * Helper function to return whether the Magnetometer is calibrated
     * In order to calibrate the Magnetometer, the robot would have to move around
     * or you would have to take the sensor off the robot and move it around. The
     * recommendation is to move it in a figure 8 type pattern. This will be very
     * difficult to do when setting up for a match.
     * @return 3 is calibrated, 0 is not calibrated, in between is in progress
     */
    private byte magCalStatus()
    {
        byte calStatus;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS;
        this.readCacheLock.lock();
        calStatus = (byte)(this.readCache[index] & 0x03);
        this.readCacheLock.unlock();
        return calStatus;
    }

    /**
     * Helper function to return whether the Accelerometer is calibrated
     * In order to calibrate the Accelerometer, the robot needs to be placed in 6 different
     * stable positions for just a couple seconds each. Ideally, 3 of positions are 3 sides
     * of the robot representing an X, Y, and Z axis. This might be a little awkward before a match.
     * @return 3 is calibrated, 0 is not calibrated, in between is in progress
     */
    private byte accCalStatus()
    {
        byte calStatus;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS;
        this.readCacheLock.lock();
        calStatus = (byte)(this.readCache[index] & 0x0C >> 2);
        this.readCacheLock.unlock();
        return calStatus;
    }

    /**
     * The Gyro is the easiest to calibrate. It just needs to be in a single stable position for a
     * couple of seconds.
     * @return 3 is calibrated, 0 is not calibrated, in between is in progress
     */
    private byte gyroCalStatus()
    {
        byte calStatus;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS;
        this.readCacheLock.lock();
        calStatus = (byte)(this.readCache[index] & 0x30);
        calStatus = (byte)(calStatus >> 4);
        this.readCacheLock.unlock();
        return calStatus;
    }

    /**
     * The System Calibration depends on all three sensors.
     * @return 3 is calibrated, 0 is not calibrated, in between is in progress
     */
    private byte sysCalStatus()
    {
        byte calStatus;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS;
        this.readCacheLock.lock();
        calStatus = (byte)(this.readCache[index] & 0xC0 >> 6);
        this.readCacheLock.unlock();
        return calStatus;
    }

    /**
     * Returns the system test status. System test is complete when equal to 0x0F
     * Bit 0 - Accelerometer
     * Bit 1 - Magnetometer
     * Bit 2 - Gyro
     * Bit 3 - Microcontroller
     * @return system test status
     */
    private byte sysTestStatus()
    {
        byte status;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS+REGISTER.SELFTEST_RESULT.bVal-REGISTER.CALIB_STAT.bVal;
        this.readCacheLock.lock();
        status = (byte)(this.readCache[index] & 0x0F);
        this.readCacheLock.unlock();
        return status;
    }

    /**
     * Currently not used but it is the System Status
     * 0 - Idle
     * 1 - System Error
     * 2 - Initializing Peripherals
     * 3 - System Initialization
     * 4 - Executing Self Test
     * 5 - Sensor Fusion, algorithm running
     * 6 - System running without fusion algorithm
     * @return system status
     */
    private byte systemStatus()
    {
        byte status;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS+REGISTER.SYS_STAT.bVal-REGISTER.CALIB_STAT.bVal;
        this.readCacheLock.lock();
        status = this.readCache[index];
        this.readCacheLock.unlock();
        return status;
    }

    /**
     * Currently not used. Reports the System Error register
     * 0 - No Error
     * 1 - Peripheral Initialization Error
     * 2 - System Initialization Error
     * 3 - Self Test result failed
     * 4 - Register map value out of range
     * 5 - Register map address out of range
     * 6 - Register map write error
     * 7 - BNO low power mode not available for selected operational mode
     * 8 - Accelerometer power mode not available
     * 9 - Fusion algorithm configuration error
     * A - Sensor configuration error
     * @return system error
     */
    private byte systemError()
    {
        byte status;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS+REGISTER.SYS_ERR.bVal-REGISTER.CALIB_STAT.bVal;
        this.readCacheLock.lock();
        status = this.readCache[index];
        this.readCacheLock.unlock();
        return status;
    }

    /**
     * The data register is a reserved register and is always 0xFF. This can always be used to
     * see if the read cache has been updated when reading the status registers
     * @return
     */
    private byte dataRegister()
    {
        byte status;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS+REGISTER.DATA_SELECT.bVal-REGISTER.CALIB_STAT.bVal;
        this.readCacheLock.lock();
        status = this.readCache[index];
        this.readCacheLock.unlock();
        return status;
    }

    /**
     * Returns the current operating mode of the BNO
     * 0 - Config Mode
     * 1 - Accelerometer Only
     * 2 - Magnetometer Only
     * 3 - Gyro Only
     * 4 - Accelerometer and Magnetometer
     * 5 - Accelerometer and Gyro
     * 6 - Magnetometer and Gyro
     * 7 - Accelerometer, Magnetometer, and Gyro (non-fusion)
     * 8 - IMU (fusion)
     * 9 - Compass (fusion)
     * 10 - M4G (fusion)
     * 11 - NDOF FMC OFF
     * 12 - NDOF
     * @return operational mode
     */
    private byte operationalMode()
    {
        byte status;
        int index;

        index = I2cController.I2C_BUFFER_START_ADDRESS+REGISTER.OPR_MODE.bVal-REGISTER.CALIB_STAT.bVal;
        this.readCacheLock.lock();
        status = (byte)(this.readCache[index]);
        this.readCacheLock.unlock();
        return status;
    }

    /**
     * Access function the current state of the BoschGyro Java Class
     * @return imuState
     */
    public States currentState() {
        return imuState;
    }

    /**
     * This routine should be called repeated in the init_loop function or you doing linear OpMode
     * in a while loop with a call to wait one hardware cycle after every call to updateState
     */
    public void updateState() {
        // Do operation based on the current state.
        switch (imuState) {
            case IMU_INIT:
                initialize();
                break;

            case IMU_WAIT_FOR_COMPLETE:
                waitForComplete();
                break;

            case IMU_CHECK_ID:
                checkId();
                break;

            case IMU_ID_VERIFY:
                verifyId();
                break;

            case IMU_RESET:
                reset();
                break;

            case IMU_RESET_VERIFY:
                resetVerify();
                break;

            case IMU_RESET_VERIFY2:
                resetVerify2();
                break;

            case IMU_WAIT_FOR_SELF_TEST:
                waitForSelfTest();
                break;

            case IMU_SET_POWER_MODE:
                setPowerMode();
                break;

            case IMU_SET_PAGE_ID:
//                setPageId();
                break;

            case IMU_SET_UNITS:
                setUnits();
                break;

            case IMU_MODE_TRANSITION:
                modeTransition();
                break;

            case IMU_MODE_VERIFY:
                verifyImuMode();
                break;

            case IMU_MODE_VERIFY2:
                verifyImuMode2();
                break;

            case IMU_VERIFY_CALIBRATION:
                verifyCalibration();
                break;

            case IMU_READ_STATUS:
                readStatus();
                break;

            case IMU_READ_STATUS2:
                readStatus2();
                break;

            case IMU_DELAY:
                delay();
                break;

            case IMU_RUNNING:
                break;
        }

    }

    /**
     * The very first state. Command the BNO to select regiser page one. This is the default
     * register anyways but this just makes sure it was picked.
     * After verification that the command was sent, verify that we are talking to a Bosch
     * BNO055 by going to the IMU_CHECK_ID state.
     */
    private void initialize() {
        Log.i("Aluminati", "Resetting IMU to its power-on state......");
        //Set the register map PAGE_ID back to 0, to make the SYS_TRIGGER register visible
        this.outboundBytes[0] = 0x00;//Sets the PAGE_ID bit for page 0 (Table 4-2)
        if (i2cWriteImmediately(1, REGISTER.PAGE_ID)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_CHECK_ID;
        }
    }

    private void delay() {
        if (runtime.time() > delayTime)
        {
            Log.i("Aluminati","Delay Time Expired: "+delayTime);
            imuState = nextImuState;
        }
    }

    /**
     * The second state for the BoschGyro class. This commands to do a read of the CHIP ID
     * After completion of the command being executed, the state is transitioned to IMU_ID_VERIFY
     */
    private void checkId() {
        Log.i("Aluminati", "Read the CHIP ID");
        // Enable the read mode to read 4 bytes
        i2cRead(REGISTER.CHIP_ID.bVal,
                4); // Reading for to get All 4 Chip IDs.
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_ID_VERIFY;
    }

    /**
     * This is the third state for the BoschGyro class. This looks at the read cache to see
     * if the chip id is set. Note, this will stay in this state forever and never come out
     * if the Chip is not found. This function needs to be updated to somehow give.
     * The OpMode should constantly read the current state of the BoschGyro and display the state
     * very telemetry to the Driver Station phone. If the state gets "stuck" then you know you
     * probably have a hardware issue.
     * After verifying the ID, the next step is to RESET the Bosch BNO055. Next state is IMU_RESET.
     */
    private void verifyId() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == bCHIP_ID_VALUE) {
            Log.i("Aluminati","Bosch IMU ID Verified");
            Log.i("Aluminati",hex(this.readCache));
            imuState = States.IMU_RESET;
        } else {
            // I don't seem to get a busy signal. My expectation
            // that the read would set it to busy until the read
            // was complete.
            Log.i("Aluminati", "Did Not Find the Bosch IMU");
        }
        this.readCacheLock.unlock();
    }

    /**
     * This is the fourth state for the BoschGyro class. This executes a RESET command to the
     * Bosch BNO055, clears the interrupt flags, and sets the IMU clock to an external clock.
     * We could also set the Self Test bit but this happens anyways during the power process.
     * We know reset is done when the Chip ID can be successfully read
     */
    private void reset() {
        //The "E" sets the RST_SYS and RST_INT bits, and sets the CLK_SEL bit,
        // to select the external IMU clock mounted on the Adafruit board (Table 4-2, and p.70). In
        // the lower 4 bits, a "1" sets the commanded Self Test bit, which causes self-test to run (p. 46)
        // Not setting bit for Self Test since this happens during power on anyways.
        // In order to do the Self Test, the chip must be in CONFIG mode
        // CONFIG mode is the default mode after power or reset

        // While in the reset state the chip id (and other registers) reads as 0xFF.
        this.outboundBytes[0] = (byte)0xE0;
        Log.i("Aluminati", "Resetting the IMU");
        if (i2cWriteImmediately(1, REGISTER.SYS_TRIGGER)) {
            imuState = States.IMU_DELAY;
            nextImuState = States.IMU_RESET_VERIFY;
            delayTime = 0.650;
            runtime.reset();
        }
    }

    /**
     * This is the fifth state of the BoschGyro class. This will start the read command to
     * read the CHIP ID. When the CHIP ID can be verified then the reset is complete.
     */
    private void resetVerify() {
        Log.i("Aluminati", "Reading the CHIP ID after Reset");
        // Enable the read mode to read 4 bytes
        i2cRead(REGISTER.CHIP_ID.bVal,
                4);
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_RESET_VERIFY2;
    }

    /**
     * This is the sixth state of the BoschGyro class. This check the read cache to see if the
     * the CHIP ID is correct. The next state is to verify that the self test has been completed.
     * Unfortunately both cannot be read at the same time because there are more than 26 registers
     * between the ID and the status registers. So it is a two step process.
     * Like before, it will state in this state forever until the Bosch ID is found. It does
     * not time out.
     */
    private void resetVerify2() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == bCHIP_ID_VALUE) {
            Log.i("Aluminati", "Found the Bosch IMU after Reset!");
            imuState = States.IMU_READ_STATUS;
            nextImuState = States.IMU_WAIT_FOR_SELF_TEST;
            Log.i("Aluminati",hex(this.readCache));
        } else {
            // I don't seem to get a busy signal. My expectation
            // that the read would set it to busy until the read
            // was complete.
            Log.i("FtcRobotController", "Did Not Find the Bosch IMU after Reset");
        }
        this.readCacheLock.unlock();
    }

    /**
     * State #7. Waits until the status of the self is complete. Once complete the power mode
     * is set.
     */
    private void waitForSelfTest() {
        if (sysTestStatus() == 0x0F)
        {
            Log.i("Aluminati","Self Test is complete");
            Log.i("Aluminati",hex(this.readCache));
            imuState = States.IMU_SET_POWER_MODE;
        }
    }

    /**
     * State #8. Write the normal power mode to BNO055. Then set the units.
     */
    private void setPowerMode() {
        Log.i("Aluminati", "Setting The Power Mode......");
        // Set the power mode
        //  0 is Normal
        //  1 is Low
        //  2 is Suspend
        this.outboundBytes[0] = 0x00; // Normal is 0, Low is 1, Suspend is 2
        if (i2cWriteImmediately(1, REGISTER.PWR_MODE)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_SET_UNITS;
        }
    }

    /**
     * State #9. Set the Units then transition to the operational mode transition to
     * IMU_MODE_TRANSITION
     */
    private void setUnits() {
        Log.i("Aluminati", "Setting The Units......");
        this.outboundBytes[0] = (byte) ((pitchmode.bVal << 7) |       // pitch angle convention
                (temperatureUnit.bVal << 4) | // temperature
                (angleunit.bVal << 2) |       // euler angle units
                (angleunit.bVal << 1) |       // gyro units, per second
                (accelunit.bVal));    // accelerometer units
        if (i2cWriteImmediately(1, REGISTER.UNIT_SEL)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_MODE_TRANSITION;
        }
    }

    /**
     * State #10. Set the IMU mode and then read status. Transition to IMU_VERIFY_MODE
     */
    private void modeTransition() {
        Log.i("Aluminati", "Setting IMU to normal IMU Operation Mode");
        this.outboundBytes[0] = SENSOR_MODE.IMU.bVal;

        if (i2cWriteImmediately(1, REGISTER.OPR_MODE)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_MODE_VERIFY;
        }
    }

    /**
     * State #11. Check that operational mode is IMU
     */
    private void verifyImuMode() {
        Log.i("Aluminati","Verifying that the mode was set");
        imuState = States.IMU_READ_STATUS;
        nextImuState = States.IMU_MODE_VERIFY2;
    }

    /**
     * State #11. Check that operational mode is IMU.
     */
    private void verifyImuMode2() {
        if (operationalMode() == SENSOR_MODE.IMU.bVal)
        {
            Log.i("Aluminati","BOSCH BNO055 is in the IMU Mode");
            imuState = States.IMU_VERIFY_CALIBRATION;
        }
    }

    /**
     * State #12. Check that the IMU Gyro is calibrated
     */
    private void verifyCalibration() {
        if (gyroCalStatus() == 3)
        {
            Log.i("Aluminati","Gyro is Calibrated");
            i2cRead(REGISTER.EULER_H_LSB.bVal,
                    20);
            initComplete = true;
            imuState = States.IMU_RUNNING;
        }
    }

    private void readStatus() {
        Log.i("Aluminati", "Reading the Status");
        // Enable the read mode to read 8 bytes
        i2cRead(REGISTER.CALIB_STAT.bVal,
                10);
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuStateStatus = nextImuState;
        nextImuState = States.IMU_READ_STATUS2;
    }

    private void readStatus2() {
        if (dataRegister() == dataRegValue)
        {
            // Address 3C is reserved and is always FF
            // Assume Status has been read when that byte is not zero
            Log.i("Aluminati", "The Status Registers");
            imuState = nextImuStateStatus;
            Log.i("Aluminati", hex(this.readCache));
        }
    }

    private void waitForComplete() {
        if (localReady == true) {
            imuState = nextImuState;
        } else {
            Log.i("Aluminati", "IMU Port is Not Ready");
        }
    }

    private void i2cRead(int register,int byteCount)
    {
        // Clear the data just to make sure something was written
        // Of course it is hard to tell if 0 was written. So if
        // 0 is an expected answer then a delay or something probably
        // needed since there is no way to tell when the device actually
        // wrote data into the cache.
        int index;
        for (index =0; index < byteCount; index++) {
            this.readCache[I2cController.I2C_BUFFER_START_ADDRESS + index] = 0;
        }
        localReady = false;
        imu.enableI2cReadMode(this.I2C_ADDRESS,
                register,
                byteCount);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
    }

    private boolean i2cWriteImmediately(int byteCount, REGISTER registerAddress){
        long rightNow = System.nanoTime(), startTime = System.nanoTime();
        int index;

        Log.i("Aluminati","Write Immediately:"+byteCount+"  "+registerAddress.bVal);

        localReady = false;
        try {
            while ((!this.imu.isI2cPortReady())
                    && (((rightNow = System.nanoTime()) - startTime) < 1000000000L)){
                Thread.sleep(10);//"Snooze" right here, until the port is ready (a good thing) OR 1 billion
                //nanoseconds pass with the port "stuck busy" (a VERY bad thing)
            }
        } catch (InterruptedException e) {
            Log.i("Aluminati", "Unexpected interrupt while \"sleeping\" in i2cWriteImmediately.");
            return false;

        }
        if ((rightNow - startTime) >= 1000000000L) return false;//Signals the "stuck busy" condition
        try {
            this.writeCacheLock.lock();
            for (index =0; index < byteCount; index++) {
                this.writeCache[I2cController.I2C_BUFFER_START_ADDRESS + index] = this.outboundBytes[index];
                //Both the read and write caches start with 5 bytes of prefix data.
            }
            Log.i("Aluminati","Writing Data");
            Log.i("Aluminati",hex(this.writeCache));
        } finally {
            this.writeCacheLock.unlock();
        }
        //The device interface object must do this, because the i2c device object CAN'T do it, in the
        //8 August 2015 beta release of the FTC SDK
        imu.enableI2cWriteMode(this.I2C_ADDRESS, registerAddress.bVal, byteCount);
        imu.setI2cPortActionFlag();  //Set the "go do it" flag
        imu.writeI2cCacheToController();

        return true;
    }

    public void portIsReady(int port) {
        this.imu.setI2cPortActionFlag();
        this.imu.readI2cCacheFromController();
        this.imu.writeI2cPortFlagOnlyToController();
        if (localReady == false)
        {
            Log.i("Aluminati","Port is Ready being set to TRUE");
        }
        localReady = true;
    }

    /**
     * Replaces the getDeviceName from HardwareDevice
     * @return string for the device name
     */
    public String getDeviceName() {

        return "BNO05 Bosch 9-Axis Orientation Sensor";
    }

    public static void throwIfModernRoboticsI2cAddressIsInvalid(int newAddress) {
        if(newAddress >= 16 && newAddress <= 126) {
            if(newAddress % 2 != 0) {
                throw new IllegalArgumentException(String.format("New I2C address %d is invalid; the address must be even.", new Object[]{Integer.valueOf(newAddress)}));
            }
        } else {
            throw new IllegalArgumentException(String.format("New I2C address %d is invalid; valid range is: %d..%d", new Object[]{Integer.valueOf(newAddress), Integer.valueOf(16), Integer.valueOf(126)}));
        }
    }

    /**
     * Replace the getVersion from HardwareDevice
     * Returns the current version
     * @return version
     */
    public int getVersion() {
        return 1;
    }

    /**
     * Replace the close from HardwareDevice
     * Does nothing
     */
    public void close() {
    }

    /**
     * Replaces the getConnectionInfo from HardwareDevice
     * Returns the port for the I2C Interface
     * @return
     */
    public String getConnectionInfo() {
        return this.imu.getConnectionInfo() + "; I2C port ?";
    }

    public void setI2cAddress(int newAddress) {
        throwIfModernRoboticsI2cAddressIsInvalid(newAddress);
        this.I2C_ADDRESS = newAddress;
    }

    public int getI2cAddress() {
        return this.I2C_ADDRESS;
    }

}

