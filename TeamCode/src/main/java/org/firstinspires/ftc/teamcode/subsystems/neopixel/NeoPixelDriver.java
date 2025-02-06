package org.firstinspires.ftc.teamcode.subsystems.neopixel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import org.apache.commons.lang3.SerializationUtils;
import java.util.Arrays;
import java.util.concurrent.locks.ReentrantLock;

@SuppressWarnings({"WeakerAccess", "unused"})
@Config
@I2cDeviceType
@DeviceProperties(name = "Adafruit I2C NeoPixel Driver", xmlTag = "NeoPixel")
public class NeoPixelDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    // Constants for I2C commands and configuration
    private static final int PIN = 0x01;
    private static final int SPEED = 0x02;
    private static final int BUF_LENGTH = 0x03;
    private static final int BUF = 0x04;
    private static final int SHOW = 0x05;
    private static final int BASE = 0x0E;
    private static final int PACKET_SIZE = 22;

    // Hardware configuration
    private static final int DEFAULT_PIN = 15;

    // Constants for LED configuration
    private static final int MAX_PIXELS = 170;  // Maximum number of pixels supported
    private static final int BYTES_PER_PIXEL = 3;  // RGB = 3 bytes per pixel
    private static final int MIN_PIXEL_INDEX = 1;  // Minimum valid pixel index (1-based)

    // Buffer for color data
    private final byte[] colorMap = new byte[MAX_PIXELS * BYTES_PER_PIXEL];
    private int currentMax = 0;

    // I2C communication buffers
    private final byte[] buffLengthData = new byte[4];  // Increased to 4 bytes to handle 16-bit length
    private byte[][] bufferData;
    private byte[][] prevBufferData;
    private int packageCount = 0;

    // Lock for thread safety
    private final ReentrantLock i2cLock = new ReentrantLock();

    /**
     * Sets the NeoPixel data pin on the microcontroller
     * @param pin Pin number to use
     */
    public void setPin(int pin) {
        byte[] data = {(byte) PIN, (byte) pin};
        deviceClient.write(BASE, data);
    }

    /**
     * Sets the color for a specific pixel
     * @param pixel Pixel index (1-based)
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @throws IllegalArgumentException if pixel index is invalid
     */
    public void setColor(int pixel, int r, int g, int b) {
        // Validate pixel index (1-based indexing)
        if (pixel < MIN_PIXEL_INDEX || pixel > MAX_PIXELS) {
            throw new IllegalArgumentException(
                    String.format("Pixel index must be between %d and %d", MIN_PIXEL_INDEX, MAX_PIXELS)
            );
        }

        // Calculate the actual position in the colorMap (adjusting for 1-based indexing)
        int realLength = (3 * pixel);

        // Update color values in the colorMap
        colorMap[realLength - 3] = (byte) g;
        colorMap[realLength - 2] = (byte) r;
        colorMap[realLength - 1] = (byte) b;

        // Update the maximum used length
        currentMax = Math.max(realLength, currentMax);

        // Calculate required number of packets
        packageCount = (int) Math.ceil(currentMax / (double) PACKET_SIZE);

        // Prepare the buffer length command (16-bit value)
        int totalLength = PACKET_SIZE * packageCount;
        buffLengthData[0] = (byte) BUF_LENGTH;
        buffLengthData[1] = 0;  // Reserved byte
        buffLengthData[2] = (byte) (totalLength & 0xFF);  // Low byte
        buffLengthData[3] = (byte) ((totalLength >> 8) & 0xFF);  // High byte

        // Initialize buffer data array
        bufferData = new byte[packageCount][PACKET_SIZE + 3];

        // Fill buffer data packets
        for (int i = 0; i < packageCount; i++) {
            bufferData[i][0] = (byte) BUF;  // Command byte
            bufferData[i][2] = (byte) (i * PACKET_SIZE);  // Offset byte

            // Copy color data, ensuring we don't exceed colorMap bounds
            for (int j = 0; j < PACKET_SIZE; j++) {
                int sourceIndex = i * PACKET_SIZE + j;
                if (sourceIndex < currentMax) {
                    bufferData[i][3 + j] = colorMap[sourceIndex];
                } else {
                    bufferData[i][3 + j] = 0;  // Pad with zeros if beyond currentMax
                }
            }
        }
    }

    /**
     * Sends the color data to the NeoPixel strip
     * Thread-safe implementation to prevent I2C conflicts
     */
    public void show() {
        i2cLock.lock();
        try {
            // Skip if no changes since last update
            if (Arrays.deepEquals(prevBufferData, bufferData)) {
                return;
            }

            // Send buffer length (16-bit)
            deviceClient.write(BASE, buffLengthData);

            // Send color data in packets
            for (int i = 0; i < packageCount; i++) {
                deviceClient.write(BASE, bufferData[i]);
            }

            // Send show command
            byte[] shower = {(byte) SHOW};
            deviceClient.write(BASE, shower);

            // Save current state
            prevBufferData = SerializationUtils.clone(bufferData);

        } finally {
            i2cLock.unlock();
        }
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

    public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);

    public NeoPixelDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        setPin(DEFAULT_PIN);
    }
}