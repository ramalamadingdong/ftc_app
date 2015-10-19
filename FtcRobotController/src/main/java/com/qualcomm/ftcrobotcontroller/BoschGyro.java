package com.qualcomm.ftcrobotcontroller;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

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

public class BoschGyro implements I2cController.I2cPortReadyCallback {

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
        IMU_SET_POWER_MODE,
        IMU_SET_PAGE_ID,
        IMU_SET_UNITS,
        IMU_SET_CRYSTAL,
        IMU_RUN_SELF_TEST1,
        IMU_RUN_SELF_TEST2,
        IMU_RUN_SELF_TEST3,
        IMU_RUN_SELF_TEST4,
        IMU_CALIBRATE_INIT,
        IMU_CALIBRATE,
        IMU_CALIBRATE_DONE,
        IMU_SET_IMU_MODE,
        IMU_RUNNING,
    }

    private final I2cDevice imu;
    public volatile int I2C_ADDRESS = 0x28 * 2;
    public volatile boolean localReady = true;
    private final int i2cBufferSize = 26; //Size of any extra buffers that will hold any incoming or outgoing cache data
    private final DeviceInterfaceModule dim;
    private final int port;
    private final byte[] readCache;
    private final Lock readCacheLock;
    private final byte[] writeCache; //This cache will hold the bytes which are to be written to the interface
    private final Lock writeCacheLock; //A lock on access to the IMU's I2C write cache
    private SENSOR_MODE sensorMode;//The operational mode to which the IMU will be set after its initial reset.
    private States imuState;
    private States nextImuState;
    private boolean initComplete;
    private byte[] outboundBytes;

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

    /** calibration data with which the BNO055 should be initialized */
    public byte[]           calibrationData     = null;

    private int numberOfRegisters = 20;
    private int readCacheOffset = REGISTER.EULER_H_MSB.bVal - I2cController.I2C_BUFFER_START_ADDRESS;

    public BoschGyro(DeviceInterfaceModule module, int physicalPort) {
        imu = new I2cDevice(module, physicalPort); //Identify the IMU with the port to
        //which it is connected on the Modern Robotics Core Device Interface Module

        this.dim = module;
        this.port = physicalPort;

        this.outboundBytes = new byte[i2cBufferSize];

        this.readCache = this.imu.getI2cReadCache();
        this.readCacheLock = this.imu.getI2cReadCacheLock();
        this.writeCache = this.imu.getI2cWriteCache();
        this.writeCacheLock = this.imu.getI2cWriteCacheLock();
        this.sensorMode = SENSOR_MODE.IMU;

        this.dim.enableI2cReadMode(physicalPort, this.I2C_ADDRESS, 4, numberOfRegisters);
        this.dim.setI2cPortActionFlag(physicalPort);
        this.dim.writeI2cCacheToController(physicalPort);
        this.dim.registerForI2cPortReadyCallback(this, physicalPort);

        this.imuState = States.IMU_INIT;
        this.nextImuState = States.IMU_INIT;
        this.initComplete = false;
    }

    public boolean initComplete() {
        return this.initComplete;
    }

    public double heading()
    {
        int headingInt;
        double h;

        this.readCacheLock.lock();

        headingInt = this.readCache[I2cController.I2C_BUFFER_START_ADDRESS]  +
                this.readCache[I2cController.I2C_BUFFER_START_ADDRESS+1] * 256;

        h = ((float)headingInt) * 1.0/16.0;

        this.readCacheLock.unlock();
        return h;
    }

    public States currentState() {
        return imuState;
    }

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

            case IMU_SET_POWER_MODE:
                setPowerMode();
                break;

            case IMU_SET_PAGE_ID:
//                setPageId();
                break;

            case IMU_SET_UNITS:
                setUnits();
                break;

            case IMU_SET_CRYSTAL:
                setCrystal();
                break;

            case IMU_RUN_SELF_TEST1:
                runSelfTest1();
                break;

            case IMU_RUN_SELF_TEST2:
                runSelfTest2();
                break;

            case IMU_RUN_SELF_TEST3:
                runSelfTest3();
                break;

            case IMU_RUN_SELF_TEST4:
                runSelfTest4();
                break;

            case IMU_CALIBRATE_INIT:
                calibrateInit();
                break;

            case IMU_CALIBRATE:
                calibrate();
                break;

            case IMU_CALIBRATE_DONE:
                calibrateDone();
                break;

            case IMU_RUNNING:
                break;
        }

    }

    private void initialize() {
        Log.i("FtcRobotController", "Resetting IMU to its power-on state......");
        //Set the register map PAGE_ID back to 0, to make the SYS_TRIGGER register visible
        this.outboundBytes[0] = 0x00;//Sets the PAGE_ID bit for page 0 (Table 4-2)
        if (i2cWriteImmediately(1, REGISTER.PAGE_ID)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_CHECK_ID;
        }
    }

    private void checkId() {
        // Clear the data just to make sure it can be read
        this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] = 0;

        Log.i("FtcRobotController", "Reading the CHIP ID");
        localReady = false;
        // Enable the read mode to read 1 byte
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.CHIP_ID.bVal,
                1);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_ID_VERIFY;
    }

    private void verifyId() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == bCHIP_ID_VALUE) {
            Log.i("FtcRobotController", "Found the Bosch IMU!");
            nextImuState = States.IMU_RESET;
            setSensorMode(SENSOR_MODE.CONFIG);
        } else {
            // I don't seem to get a busy signal. My expectation
            // that the read would set it to busy until the read
            // was complete.
            Log.i("FtcRobotController", "Did Not Find the Bosch IMU");

        }
        this.readCacheLock.unlock();
    }

    private void reset() {
        //The "E" sets the RST_SYS and RST_INT bits, and sets the CLK_SEL bit,
        // to select the external IMU clock mounted on the Adafruit board (Table 4-2, and p.70). In
        // the lower 4 bits, a "1" sets the commanded Self Test bit, which causes self-test to run (p. 46)

        // While in the reset state the chip id (and other registers) reads as 0xFF.
        this.outboundBytes[0] = (byte)0xE1;
        Log.i("FtcRobotController", "Resetting the IMU, Run Self Tests");
        if (i2cWriteImmediately(1, REGISTER.SYS_TRIGGER)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_RESET_VERIFY;
        }
    }

    private void resetVerify() {
        // Clear the data just to make sure it can be read
        this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] = 0;

        Log.i("FtcRobotController", "Reading the CHIP ID after Reset");
        localReady = false;
        // Enable the read mode to read 1 byte
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.CHIP_ID.bVal,
                1);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_RESET_VERIFY2;
    }

    private void resetVerify2() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == bCHIP_ID_VALUE) {
            Log.i("FtcRobotController", "Found the Bosch IMU after Reset!");
            imuState = States.IMU_SET_POWER_MODE;
        } else {
            // I don't seem to get a busy signal. My expectation
            // that the read would set it to busy until the read
            // was complete.
            Log.i("FtcRobotController", "Did Not Find the Bosch IMU after Reset");
        }
        this.readCacheLock.unlock();
    }

    private void setPowerMode() {
        Log.i("FtcRobotController", "Setting The Power Mode......");
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

    private void setUnits() {
        Log.i("FtcRobotController", "Setting The Units......");
        this.outboundBytes[0] = (byte) ((pitchmode.bVal << 7) |       // pitch angle convention
                (temperatureUnit.bVal << 4) | // temperature
                (angleunit.bVal << 2) |       // euler angle units
                (angleunit.bVal << 1) |       // gyro units, per second
                (accelunit.bVal));    // accelerometer units
        if (i2cWriteImmediately(1, REGISTER.UNIT_SEL)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_RUN_SELF_TEST3;
        }
    }

    private void setCrystal() {
        Log.i("FtcRobotController", "Setting The Crystal......");
        //Set the register map PAGE_ID back to 0
        if (useExternalCrystal)
        {
            this.outboundBytes[0] = (byte)(0x80);
        }
        else
        {
            this.outboundBytes[0] = 0x00;
        }
        if (i2cWriteImmediately(1, REGISTER.SYS_TRIGGER)) {
            imuState = States.IMU_WAIT_FOR_COMPLETE;
            nextImuState = States.IMU_RUN_SELF_TEST1;
        }
    }

    private void runSelfTest1() {
        // Run a self test. This appears to be a necessary step in order for the
        // sensor to be able to actually be used.

        // Initially read the SYS_TRIGGER
        // Clear the data just to make sure it can be read
        this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] = 0;

        Log.i("FtcRobotController", "Reading the SYS_TRIGGER");
        localReady = false;
        // Enable the read mode to read 1 byte
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.SYS_TRIGGER.bVal,
                1);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_RUN_SELF_TEST2;
    }

    private void runSelfTest2() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == 0) {
            Log.i("FtcRobotController", "Waiting to Read SYS_TRIGGER");
        } else {
            Log.i("FtcRobotController", "Read The SYS TRIGGER - Now write it back out");
            //Set the register map PAGE_ID back to 0
            this.outboundBytes[0] = (byte) (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] | 0x01);
            if (i2cWriteImmediately(1, REGISTER.PAGE_ID)) {
                imuState = States.IMU_WAIT_FOR_COMPLETE;
                nextImuState = States.IMU_RUN_SELF_TEST3;
            }
        }
        this.readCacheLock.unlock();
    }

    private void runSelfTest3() {
        // Initially read the Self Test Results
        // Clear the data just to make sure it can be read
        this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] = 0;

        Log.i("FtcRobotController", "Reading the Self Test Results");
        localReady = false;
        // Enable the read mode to read 1 byte
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.SELFTEST_RESULT.bVal,
                1);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_RUN_SELF_TEST4;
    }

    private void runSelfTest4() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == 0) {
            Log.i("FtcRobotController", "Waiting to Read Self Test Results");
        } else {
            Log.i("FtcRobotController", "Read The Self Test Results");
            if ((this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] & 0x0F) == 0x0F) {
                Log.i("FtcRobotController", "Self Test Passed - Enabling IMU Mode");
                nextImuState = States.IMU_CALIBRATE_INIT;
                setSensorMode(SENSOR_MODE.IMU);
            }
            else
            {
                Log.i("FtcRobotController","Self Test Failed");
            }
        }
        this.readCacheLock.unlock();
    }


    private void calibrateInit() {
        // Clear the data just to make sure data was read in
        this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] = 0;

        Log.i("FtcRobotController", "Reading the calibration status");
        localReady = false;
        // Enable the read mode to read 1 byte
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.CALIB_STAT.bVal,
                1);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        imuState = States.IMU_WAIT_FOR_COMPLETE;
        nextImuState = States.IMU_CALIBRATE;
    }

    private void calibrate() {
        this.readCacheLock.lock();
        if (this.readCache[I2cController.I2C_BUFFER_START_ADDRESS] == 0) {
            Log.i("FtcRobotController", "Waiting for calibration data");
        } else {
            Log.i("FtcRobotController", "Calibration Data:" + this.readCache[I2cController.I2C_BUFFER_START_ADDRESS]);
            imuState = States.IMU_CALIBRATE_DONE;
        }
        this.readCacheLock.unlock();

    }

    private void calibrateDone() {
        Log.i("FtcRobotController", "Calibration Done - Now Running ");
        localReady = false;
        this.dim.enableI2cReadMode(this.port,
                this.I2C_ADDRESS,
                REGISTER.EULER_H_LSB.bVal,
                20);
        imu.setI2cPortActionFlag();//Set this flag to do the next read
        imu.writeI2cCacheToController();
        initComplete = true;
        imuState = States.IMU_RUNNING;
    }

    private void waitForComplete() {
//        if (this.imu.isI2cPortReady()) {
        if (localReady == true) {
            Log.i("FtcRobotController", "IMU Port is Ready");
            imuState = nextImuState;
        } else {
            Log.i("FtcRobotController", "IMU Port is Not Ready");
        }
    }

    private void setSensorMode(SENSOR_MODE mode)
    {
        sensorMode = mode;
        this.outboundBytes[0] = (byte) (mode.bVal & 0x0F);

        Log.i("FtcRobotController", "Setting the Operational Mode to" + mode);
        i2cWriteImmediately(1, REGISTER.OPR_MODE);
        imuState = States.IMU_WAIT_FOR_COMPLETE;
    }

    private boolean i2cWriteImmediately(int byteCount, REGISTER registerAddress){
        long rightNow = System.nanoTime(), startTime = System.nanoTime();
        int index;

        localReady = false;
        try {
            while ((!this.imu.isI2cPortReady())
                    && (((rightNow = System.nanoTime()) - startTime) < 1000000000L)){
                Thread.sleep(10);//"Snooze" right here, until the port is ready (a good thing) OR 1 billion
                //nanoseconds pass with the port "stuck busy" (a VERY bad thing)
            }
        } catch (InterruptedException e) {
            Log.i("FtcRobotController", "Unexpected interrupt while \"sleeping\" in i2cWriteImmediately.");
            return false;

        }
        if ((rightNow - startTime) >= 1000000000L) return false;//Signals the "stuck busy" condition
        try {
            this.writeCacheLock.lock();
            for (index =0; index < byteCount; index++) {
                this.writeCache[I2cController.I2C_BUFFER_START_ADDRESS + index] = this.outboundBytes[index];
                //Both the read and write caches start with 5 bytes of prefix data.
            }
        } finally {
            this.writeCacheLock.unlock();
        }
        //The device interface object must do this, because the i2c device object CAN'T do it, in the
        //8 August 2015 beta release of the FTC SDK
        this.dim.enableI2cWriteMode(this.port, this.I2C_ADDRESS, registerAddress.bVal, byteCount);
        imu.setI2cPortActionFlag();  //Set the "go do it" flag
        imu.writeI2cPortFlagOnlyToController();

        return true;
    }

    public void portIsReady(int port) {
        this.dim.setI2cPortActionFlag(port);
        this.dim.readI2cCacheFromController(port);
        this.dim.writeI2cPortFlagOnlyToController(port);
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
        return this.dim.getConnectionInfo() + "; I2C port " + this.port;
    }

    public void setI2cAddress(int newAddress) {
        throwIfModernRoboticsI2cAddressIsInvalid(newAddress);
        this.I2C_ADDRESS = newAddress;
    }

    public int getI2cAddress() {
        return this.I2C_ADDRESS;
    }

}

