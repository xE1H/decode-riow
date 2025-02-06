package org.firstinspires.ftc.teamcode.subsystems.neopixel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.apache.commons.lang3.SerializationUtils;

import java.util.Arrays;

@SuppressWarnings({"WeakerAccess", "unused"})
@Config
@I2cDeviceType
@DeviceProperties(name = "Adafruit I2C NeoPixel Driver", xmlTag = "NeoPixel")
public class NeoPixelDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final int pin = 15;
    private static final int PACKET_SIZE = 22;
    private static final int BASE = 0x0E;
    private static final int PIN = 0x01;
    private static final int SPEED = 0x02;
    private static final int BUF_LENGTH = 0x03;
    private static final int BUF = 0x04;
    private static final int SHOW = 0x05;


    int currentMax = 0;
    byte[] colorMap = new byte[170 * 3];


    public void setPin(int pin) {
        byte[] data = {(byte) PIN, (byte) pin};
        deviceClient.write(BASE, data);
    }


    byte[] buffLengthData = new byte[3];
    byte[][] bufferData;
    byte[][] prevBufferData;
    int packageCount = 0;

    public void setColor(int pixel, int r, int g, int b) {
        int realLength = (3 * pixel);
        buffLengthData[0] = (byte) BUF_LENGTH;
        buffLengthData[1] = 0;
        colorMap[realLength - 3] = (byte) g;
        colorMap[realLength - 2] = (byte) r;
        colorMap[realLength - 1] = (byte) b;
        currentMax = Math.max(realLength, currentMax);
        packageCount = (int)Math.ceil(currentMax/(double)PACKET_SIZE);
        bufferData = new byte[packageCount][PACKET_SIZE+3];

        buffLengthData[2] = (byte) (PACKET_SIZE*packageCount);


        for(int i = 0; i<packageCount;i++)
        {

            bufferData[i][0] = (byte) BUF;
            bufferData[i][2] = (byte) (i*22);

            for (int j = 0; j < PACKET_SIZE; j++) {
                bufferData[i][3 + j] = colorMap[i*PACKET_SIZE+j];
            }

        }




    }


    public void show() {
        if(Arrays.deepEquals(prevBufferData, bufferData)) return;

        byte[] shower = {(byte) SHOW};
        deviceClient.write(BASE, buffLengthData);
        for(int i = 0; i<packageCount;i++)
        {
            deviceClient.write(BASE, bufferData[i]);
        }
        deviceClient.write(BASE, shower);

        prevBufferData = SerializationUtils.clone(bufferData);


    }


    @Override
    public Manufacturer getManufacturer() {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {

        return "Adafruit I2C NeoPixel Driver";
    }

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);

    public NeoPixelDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        setPin(pin);
    }
}