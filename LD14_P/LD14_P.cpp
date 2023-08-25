#include "LD14_P.h"

uint8_t LD14_P::CalCRC8(uint8_t *p, uint8_t len){
  uint8_t crc = 0;
  uint16_t i;
  for (i = 0; i < len; i++){
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

void LD14_P::polarToCartesion(float angle, uint16_t radius, int&x, int &y){
  float rad = angle * M_PI / 180.0;
  x = radius * cos(rad);
  y = radius * sin(rad);
}

void LD14_P::parseData(uint8_t *dataBuff, uint8_t dataLength) {
  
  float startAngle = (dataBuff[START_ANGLE_I] + (dataBuff[START_ANGLE_I + 1] << 8)) / 100;
  float endAngle   = (dataBuff[END_ANGLE_I] + (dataBuff[END_ANGLE_I + 1] << 8)) / 100;

  if(endAngle <= startAngle){
    //TODO: Need to calculate the values when endAngle is less than startAngle
    return; 
  }

  float angle_step = (endAngle - startAngle) / (POINT_PER_PACK - 1);

  for(uint8_t i= 0; i<POINT_PER_PACK; i++){
    lidar_points[i].angle = startAngle + (angle_step * i);
  }

  uint8_t n = 0;
  for (uint8_t i = DATA_START_I; i < (DATA_START_I + DATA_SIZE ); i+=3) {
    // Serial.print("i: ");
    // Serial.print(i);
    // Serial.print(", val: ");
    // Serial.println((dataBuff[i] + (dataBuff[i + 1] << 8)), HEX);

    lidar_points[n++].distance = (dataBuff[i] + (dataBuff[i + 1] << 8));
  }

  // Serial.print("Start: ");
  // Serial.print(startAngle);
  // Serial.print(", end: ");
  // Serial.print(endAngle);
  // Serial.println(", Points:");
  // for(uint8_t i=0; i<POINT_PER_PACK; i++){
  //   Serial.print(lidar_points[i].distance);
  //   Serial.print(", ");
  // }
  // Serial.println("\r\n------");
  
  return;
}

bool LD14_P::update(){
  if(Serial2.available()){
    uint8_t startByte = Serial2.read();

    if(startByte == 0x54){
      uint8_t dataLength = Serial2.read();
      
      if(DATASIZE != dataLength){
        // Serial.print(dataLength);
        // Serial.println(", Invalid data length");
        return false;
      }
      uint8_t packet[PACKETSIZE] = {0};

      uint8_t dataBytes[DATASIZE + 1]; // +1 for CRC bit
      Serial2.readBytes(dataBytes, DATASIZE + 1);

      // Reconstruct all data into a packet 
      packet[0] = startByte;
      packet[1] = dataLength;
      memcpy(packet + 2, dataBytes, DATASIZE + 1);

      // Serial.println("Raw Data: ");
      // for(uint8_t i=0; i<PACKETSIZE; i++){
      //   Serial.print(packet[i], HEX);
      //   Serial.print(", ");
      // }
      // Serial.println("\r\n------");

      uint8_t claculated_crc = CalCRC8(packet, PACKETSIZE - 1); // exclude the CRC bit
      if(packet[PACKETSIZE - 1] != claculated_crc){
        // Serial.print(claculated_crc);
        // Serial.print(" : ");
        // Serial.print(packet[PACKETSIZE - 1]);
        // Serial.println(", Invalid CRC");
        return false;
      }

      parseData(packet, PACKETSIZE);

      return true; 
    }
  }
  return false;

}

/**
 * @brief This function checks if there is any collition in the rectangular 
 *  area in front of the lidar with "midAngle" degrees as the center
 * This function should be called after "update()" returns true
 * 
 * @param height Height of the rectangular area infront of the lidar in mm
 * @param width  Width of the rectangular area infront of the lidar in mm
 * @param axis   Axis where the middle of the rectangular area lies takes 0, 1, 2, 3
 * @return true if there is any collision in the area
 * @return false if there is no collision in the area
 */
bool LD14_P::checkCollisionRect(uint16_t height, uint16_t width, uint8_t axis){

  int x1 = 0;
  int x2 = 0;
  int y1 = 0;
  int y2 = 0;
  int pointX = 0;
  int pointY = 0;

  width = width/2;
  if(axis == 0){
    x1 = width;
    y1 = height;
    x2 = 0;
    y2 = -1 * width;
  } else
  if(axis == 1){
    x1 = width;
    y1 = -1 * height;
    x2 = -1 * width;
    y2 = 0;
  } else
  if(axis == 2){
    x1 = -1 * height;
    y1 = -1 * width;
    x2 = 0;
    y2 = width;
  } else
  if(axis == 3){
    x1 = -1 * width;
    y1 = height;
    x2 = width;
    y2 = 0;
  } else{
    Serial.println("Invalid axis");
    return false;
  }

  uint8_t triggers = 0;
  for(uint8_t i= 0; i<POINT_PER_PACK; i++){
    if(lidar_points[i].distance > MIN_DETECTION_DIST && lidar_points[i].distance < MAX_DETECTION_DIST){
      polarToCartesion(lidar_points[i].angle, lidar_points[i].distance, pointX, pointY);

      // if(lidar_points[i].angle < 190 && lidar_points[i].angle > 170){
      //   Serial.print("("); Serial.print(x1); Serial.print(", "); Serial.print(y1); Serial.print(") | ");
      //   Serial.print("("); Serial.print(x2); Serial.print(", "); Serial.print(y2); Serial.print(")");
      //   Serial.println("");
      //   Serial.print("angle: ");
      //   Serial.print(lidar_points[i].angle);
      //   Serial.print(", X: ");
      //   Serial.print(pointX);
      //   Serial.print(", Y: ");
      //   Serial.print(pointY);
      //   Serial.println("\r\n---");
      // }
      
      if (pointX > x1 && pointX < x2 && pointY > y1 && pointY < y2){
        triggers++;
        if(triggers >= MIN_TRIGGER_POINTS){
          // Serial.print("("); Serial.print(x1); Serial.print(", "); Serial.print(y1); Serial.print(") | ");
          // Serial.print("("); Serial.print(x2); Serial.print(", "); Serial.print(y2); Serial.print(")");
          // Serial.println("");
          // Serial.print("angle: ");
          // Serial.print(lidar_points[i].angle);
          // Serial.print(", Dist: ");
          // Serial.print(lidar_points[i].distance);
          // Serial.print(", X: ");
          // Serial.print(pointX);
          // Serial.print(", Y: ");
          // Serial.print(pointY);
          // Serial.println("\r\n---");

          return true;
        }
            
      }
    }



    
  }

  return false;
}

/**
 * @brief This function checks if there is any collition in the circular area
 *  infront of the lidar from a start angle to an end angle
 * This function should be called after "update()" returns true
 * 
 * @param distance Trigger distance from the lidar in mm
 * @param triggerAngleStart Trigger start angle in degree
 * @param triggerAngleEnd Trigger end angle in degree
 * @return true if there is any collision
 * @return false if there is no collision
 */
bool LD14_P::checkCollision(uint16_t distance, float triggerAngleStart, float triggerAngleEnd){
  for(uint8_t i=0; i<POINT_PER_PACK; i++){
    if(triggerAngleStart < lidar_points[i].angle && lidar_points[i].angle < triggerAngleEnd){
      if(lidar_points[i].distance <= distance){
        if(lidar_points[i].distance > MIN_DETECTION_DIST && lidar_points[i].distance < MAX_DETECTION_DIST){
          return true;
        }
      }
    }
  }
  return false;
}
  
