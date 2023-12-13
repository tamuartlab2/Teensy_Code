/*
	Name: main.cpp
	Created: 11/28/2021
	Author:	Yuan Wei
	Last Modified: 12/7/2023 by Yuan Wei
*/

#include "main.h"

XBee xbee = XBee();
Adafruit_GPS GPS(&GPSSerial);
IntervalTimer IMUGPS;
IntervalTimer GPSDATA;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
RGBLED RGBLED1 = RGBLED(2, 3, 4);

//uint8_t gpsFlag = 1;
//uint8_t imuFlag = 1;
uint8_t imuCheck = 1;
uint8_t LED_blink = 0;
uint8_t LED_blink_status = 0;
double gpsLatitude = 0;
double gpsLongitude = 0;
double degreeTransfer = 1 / 60.0;
uint8_t GPSFix = 0;
uint8_t num = 0;
uint8_t sys, gyro, accel, magn = 0;
uint8_t LEDR, LEDG, LEDB = 0;
uint8_t WD = 0;
float degreeToRadian = 71 / 4068.0;
unsigned long time_k1;
unsigned long time_k;
adafruit_bno055_offsets_t bno_calibrationData = {-1,-1,-1,-1,0,0,0,51,12,1000,52};
int eeAddress = 0;
uint8_t writing_EEPROM = 0;

imu::Quaternion quat;
// sensors_event_t orientationData, angVelocityData, linearAccelData; //, magnetometerData, accelerometerData, gravityData;
imu::Vector<3> vectorEular;
imu::Vector<3> vectorGyroscope;
imu::Vector<3> vectorLinearAccel;

void setup() 
{
    Serial.begin(115200);
    //Xbee Serial
    Serial2.begin(9600);
    xbee.setSerial(Serial2);

    //set GPS init
    GPS.begin(9600);
    IMUGPS.begin(IMUGPSISR, IMUPeriod);  //100Hz
    IMUGPS.priority(128);
    GPSDATA.begin(GPSDATAISR, GPSPeriod); //1000Hz
    GPSDATA.priority(100);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
    if(!bno.begin())
    {
        imuCheck = 0;
        LEDR = 255;
        LEDG = 0;
        LEDB = 0;
        RGBLED1.setColor(LEDR, LEDG, LEDB); //red color     
    }
    else
    {
        LEDR = 0;
        LEDG = 255;
        RGBLED1.setColor(LEDR, LEDG, LEDB); //green color
        EEPROM.get(eeAddress, bno_calibrationData);
        bno.setSensorOffsets(bno_calibrationData);
    }
    delay(100);
    //bno.setExtCrystalUse(true);
    time_k1 = millis();
    time_k = millis();
}

void loop() 
{
    // unsigned long time1 = millis();
    time_k = millis();

    if (time_k - time_k1 > 1000)
    {
        time_k1 = time_k;
        if (LED_blink == 1)
        {
            if (LED_blink_status == 1)
            {
                LED_blink_status = 0;
                RGBLED1.setColor(LEDR, LEDG, LEDB);
            }
            else
            {
                LED_blink_status = 1;
                RGBLED1.setColor(0, 0, 0);
            }
            // bno.setSensorOffsets(bno_calibrationData);
        }
        else
        {
            RGBLED1.setColor(LEDR, LEDG, LEDB);
        }
    }

    //check serial
    if (Serial.available() > 0)
    {
        uint8_t CMDData = Serial.read();
        //feed watchdog
        if (CMDData == 0x01)
        {
            WD = 1;
        }
        //xbee data to be sent
        else if (CMDData == 0x03)
        {
            uint8_t sizeofData = Serial.read();
            uint8_t count = 0;
            uint8_t xbeePayloadTX[sizeofData];
            while (sizeofData != 0)
            {
              xbeePayloadTX[count] = Serial.read();
              count ++;
              sizeofData --;
            }
            Tx16Request tx = Tx16Request(0xFFFF, xbeePayloadTX, sizeof(xbeePayloadTX));
            xbee.send(tx);
            //Serial.println("sent");
        }
        //bno055 local calibration cmd
        else if (CMDData == 0x04)
        {
            EEPROM.get(eeAddress, bno_calibrationData);
            bno.setSensorOffsets(bno_calibrationData);
        }
    }
    adafruitGPS();
    bnoIMU();

    xbee.readPacket();
    //receive xbee data
    if (xbee.getResponse().isAvailable()) 
    {
        if (xbee.getResponse().getApiId() == RX_16_RESPONSE) 
        {
            Rx16Response rx16 = Rx16Response();
            xbee.getResponse().getRx16Response(rx16);
            Serial.write(0x03);       //send cmd byte
            Serial.write(rx16.getDataLength());       //send length of the data
            for (uint8_t i = 0; i < rx16.getDataLength(); i++)
            {
              Serial.write(rx16.getData(i));
            }

            Serial.write(rx16.getRssi());    //this is an unsigned number, add negative ahead when you deal with RSSI, Unit: dBm
        }
    }

    //RGBLED1.setColor(LEDR, LEDG, LEDB);    
    // unsigned long time2 = millis();
    // unsigned long dt = time2 - time1;
    // Serial.println(dt);
}

double dm_dd(double loc, char dir)
{
    double result = (int)(loc * 0.01);
    result += (loc - (result * 100)) * degreeTransfer;
    if (dir == 'S' || dir == 'W')
        result = -result;
    return result;
}

void adafruitGPS()
{
    if (GPS.newNMEAreceived())
    {
        GPS.parse(GPS.lastNMEA());
        gpsLatitude = dm_dd(GPS.latitude, GPS.lat);
        gpsLongitude = dm_dd(GPS.longitude, GPS.lon);
        //gpsLatitude = GPS.latitudeDegrees;
        //gpsLongitude = GPS.longitudeDegrees;
        GPSFix = GPS.fix;
        if (GPSFix)
        {
            LEDB = 0;
        }
        else
        {
            LEDB = 255;
        }

        // Serial.print(gpsLatitude);
        // Serial.print(' ');
        // Serial.println(gpsLongitude);
        // Serial.print("Satellite number");
        // Serial.println(GPS.satellites);
        // Serial.print("Fix quality:");
        // Serial.println(GPS.fixquality);

    }
}

void bnoIMU()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - degrees/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //Display the floating point data 
    //imu::Vector<3> vectorEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Serial.print("X: ");
    //Serial.print(euler.x());
    //Serial.print(" Y: ");
    //Serial.print(euler.y());
    //Serial.print(" Z: ");
    //Serial.print(euler.z());
    //Serial.print("\t\t");

    if (!imuCheck)
    {
        if(bno.begin())
        {
            imuCheck = 1; 
            LEDR = 0;
            LEDG = 255;
            EEPROM.get(eeAddress, bno_calibrationData);
            bno.setSensorOffsets(bno_calibrationData);
        }
        delay(100);
    }
    
    quat = bno.getQuat();
    vectorEular = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    vectorGyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    vectorLinearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    bno.getCalibration(&sys, &gyro, &accel, &magn);
    // Serial.print("sys:");
    // Serial.println(sys);
    // Serial.print("gyro:");
    // Serial.println(gyro);
    // Serial.print("accel:");
    // Serial.println(accel);

    if (bno.isFullyCalibrated() && writing_EEPROM == 0)
    {
        bno.getSensorOffsets(bno_calibrationData);
        adafruit_bno055_offsets_t check_bno_calibrationData;
        EEPROM.get(eeAddress, check_bno_calibrationData);

        //Protect EEPROM
        if ( (bno_calibrationData.mag_offset_x != check_bno_calibrationData.mag_offset_x) 
          || (bno_calibrationData.mag_offset_y != check_bno_calibrationData.mag_offset_y) 
          || (bno_calibrationData.mag_offset_z != check_bno_calibrationData.mag_offset_z))
        {
            EEPROM.put(eeAddress, bno_calibrationData);
        }

        // else
        // {
        //     Serial.println("The stored data are the same as the new data! ");
        // }
        writing_EEPROM = 1;
        // Serial.println("Save calibration to the EEPROM.");
    }

    if (sys == 0 || gyro == 0)
    {
        LED_blink = 1;
    }
    else
    {
        LED_blink = 0;
    }

    if((quat.x() == 0) && (quat.y() == 0) && (quat.z() == 0) && (quat.w() == 0))
    {
        imuCheck = 0;
        LEDR = 255;
        LEDG = 0;
        LEDB = 0;
      
    }
}

void IMUGPSISR()
{
    if (WD==1)
    {
        sendMSG();
        WD = 0;
    }

}

void GPSDATAISR()
{
    char c = GPS.read();
    //Serial.write(c);
}

void sendMSG()
{
    while (num == 100)  //update GPS at 1 HZ rate
    {
        num = 0;
        /*
        Serial.print(2);
        Serial.print(" ");
        Serial.print(GPSFix);
        Serial.print(" ");
        Serial.print(gpsLatitude, 8);
        Serial.print(" ");
        Serial.println(gpsLongitude, 8);   
        */
        u_int8_t gpsLatitudeByte[8], gpsLongitudeByte[8];
        double2Byte(gpsLatitude, gpsLatitudeByte);
        double2Byte(gpsLongitude, gpsLongitudeByte);
        Serial.write(2);
        Serial.write(GPSFix);
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.write(gpsLatitudeByte[i]);
        }
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.write(gpsLongitudeByte[i]);
        }
         
    }
    num++;

    struct imu_data_structure imu_data;
    uint16_t size_imu_data = sizeof(struct imu_data_structure);
    imu_data.sensorType = 1;
    imu_data.sys = sys;
    imu_data.gyro = gyro;
    imu_data.accel = accel;
    imu_data.quat_x = quat.x();
    imu_data.quat_y = quat.y();
    imu_data.quat_z = quat.z();
    imu_data.quat_w = quat.w();
    // imu_data.Gyroscope_x = angVelocityData.gyro.x;
    // imu_data.Gyroscope_y = angVelocityData.gyro.y;
    // imu_data.Gyroscope_z = angVelocityData.gyro.z;
    // imu_data.LinearAccel_x = linearAccelData.gyro.x;
    // imu_data.LinearAccel_y = linearAccelData.gyro.y;
    // imu_data.LinearAccel_z = linearAccelData.gyro.z;
    // imu_data.compass = orientationData.orientation.x;
    imu_data.Gyroscope_x = vectorGyroscope.x()*degreeToRadian;
    imu_data.Gyroscope_y = vectorGyroscope.y()*degreeToRadian;
    imu_data.Gyroscope_z = vectorGyroscope.z()*degreeToRadian;
    imu_data.LinearAccel_x = vectorLinearAccel.x();
    imu_data.LinearAccel_y = vectorLinearAccel.y();
    imu_data.LinearAccel_z = vectorLinearAccel.z();
    imu_data.compass = vectorEular.x();
    Serial.write((char*)&imu_data, size_imu_data); 

    // Serial.print(angVelocityData.gyro.z, 8);
    // Serial.print(' ');
    // Serial.println(vectorGyroscope.z()*degreeToRadian, 8); 

}
