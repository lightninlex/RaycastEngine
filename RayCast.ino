#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_heap_caps.h>
#include "roomMaps.h"
#include "config.h"

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define JOYX 34
#define JOYY 35

// Function to convert RGB to 16-bit color
uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
#define TFT_GREY RGB(128, 128, 128)
#define TFT_CHARCOAL RGB(54, 69, 79)

float playerX = 1.0;
float playerY = 1.0;
float playerAngle = 0.0;
float playerSpeed = 0.1;
float FOV = PI / 4;

TFT_eSPI tft = TFT_eSPI();
uint16_t *screenBuffer;
const int (*currentRoomMap)[MAP_WIDTH] = roomMapA;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);

  screenBuffer = (uint16_t *)heap_caps_malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (screenBuffer == NULL) {
    Serial.println("Failed to allocate PSRAM for screen buffer");
    while (1);
  }
}

void win() {
  for(;;) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(0, 0);
    tft.println("You");
    tft.println("Win!");
    delay(100000);
  }
}

void loop() {
  handleInput();
  bool sceneUpdated = renderScene();
  if (sceneUpdated) {
    tft.pushImage(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, screenBuffer); // Update the screen at once
  }

  if (currentRoomMap == roomMapA && playerX >= 30.0 && playerY >= 30.0) {
    currentRoomMap = roomMapB;
    playerX = 1.0;
    playerY = 1.0;
  } else if (currentRoomMap == roomMapB && playerX >= 30.0 && playerY >= 30.0) {
    win();
  }
}

bool handleInput() {
  int deadzone = 150;
  int centerX = 1915;
  int centerY = 1870;
  bool needsUpdate = false;
  
  int joyX = analogRead(JOYX);
  int joyY = analogRead(JOYY);

  if (joyY >= centerY + deadzone) {
    float newX = playerX + cos(playerAngle) * playerSpeed;
    float newY = playerY + sin(playerAngle) * playerSpeed;
    if (canMoveTo(newX, newY)) {
      playerX = newX;
      playerY = newY;
      needsUpdate = true;
    }
  }
  if (joyY <= centerY - deadzone) {
    float newX = playerX - cos(playerAngle) * playerSpeed;
    float newY = playerY - sin(playerAngle) * playerSpeed;
    if (canMoveTo(newX, newY)) {
      playerX = newX;
      playerY = newY;
      needsUpdate = true;
    }
  }
  if (joyX <= centerX - deadzone) {
    playerAngle -= 0.1;
    if (playerAngle < 0) playerAngle += 2 * PI;
    needsUpdate = true;
  }
  if (joyX >= centerX + deadzone) {
    playerAngle += 0.1;
    if (playerAngle > 2 * PI) playerAngle -= 2 * PI;
    needsUpdate = true;
  }
  return needsUpdate;
}

bool canMoveTo(float x, float y) {
  int mapX = (int)x;
  int mapY = (int)y;
  if (mapX < 0 || mapX >= MAP_WIDTH || mapY < 0 || mapY >= MAP_HEIGHT) {
    return false;
  }
  return currentRoomMap[mapY][mapX] == 0;
}

bool renderScene() {
  static float oldPlayerX = -1, oldPlayerY = -1, oldPlayerAngle = -1;

  // Only redraw if necessary
  if (oldPlayerX == playerX && oldPlayerY == playerY && oldPlayerAngle == playerAngle) {
    return false;
  }

  oldPlayerX = playerX;
  oldPlayerY = playerY;
  oldPlayerAngle = playerAngle;

  memset(screenBuffer, 0, DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t)); // Clear the buffer

  float wallDistances[DISPLAY_WIDTH];

  for (int x = 0; x < DISPLAY_WIDTH; x++) {
    float rayAngle = playerAngle + (FOV * (x - DISPLAY_WIDTH / 2)) / DISPLAY_WIDTH;
    wallDistances[x] = castRay(rayAngle, x);
  }

  // Draw the walls and floor based on wall distances
  for (int x = 0; x < DISPLAY_WIDTH; x++) {
    float distance = wallDistances[x];
    int wallHeight = min(int(DISPLAY_HEIGHT / distance), DISPLAY_HEIGHT);
    int wallTop = (DISPLAY_HEIGHT - wallHeight) / 2;

    // Draw the wall
    for (int y = wallTop; y < wallTop + wallHeight; y++) {
      uint16_t color = calculateWallColor(distance);
      screenBuffer[y * DISPLAY_WIDTH + x] = color; // Fill the line in the buffer
    }

    // Draw the floor
    for (int y = wallTop + wallHeight; y < DISPLAY_HEIGHT; y++) {
      float floorDistance = (float)DISPLAY_HEIGHT / (2.0f * y - DISPLAY_HEIGHT);
      uint16_t color = calculateFloorColor(floorDistance);
      screenBuffer[y * DISPLAY_WIDTH + x] = color; // Fill the line in the buffer
    }
  }

  return true; // Indicate that the scene was updated
}

uint16_t calculateFloorColor(float distance) {
  int brightness = max(0, 255 - (int)(distance * 10)); // Adjust the factor to achieve the correct gradient
  brightness = min(255, brightness);
  return RGB(brightness, brightness, brightness); // Returns a grayscale color
}

uint16_t calculateWallColor(float distance) {
  int brightness = max(0, 255 - (int)(distance * 12));
  brightness = min(255, brightness);
  return RGB(brightness, brightness, brightness); // Returns a grayscale color
}

float castRay(float angle, int column) {
  float rayX = playerX;
  float rayY = playerY;
  float distanceToWall = 0;
  bool hitWall = false;

  while (!hitWall) {
    rayX += cos(angle) * 0.1;
    rayY += sin(angle) * 0.1;

    int mapX = int(rayX);
    int mapY = int(rayY);

    if (mapX < 0 || mapX >= MAP_WIDTH || mapY < 0 || mapY >= MAP_HEIGHT) {
      hitWall = true;
      distanceToWall = DISPLAY_WIDTH; // Large distance to represent out of bounds
    } else if (currentRoomMap[mapY][mapX] == 1) {
      hitWall = true;
      distanceToWall = sqrt((rayX - playerX) * (rayX - playerX) + (rayY - playerY) * (rayY - playerY));
    }
  }

  return distanceToWall;
}
