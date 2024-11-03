#include "Arduino.h"

// Placeholder sum function (to be defined properly as per your logic)
int sum(int arr[], int size) {
    int s = 0;
    for (int i = 0; i < size; ++i) {
        s += arr[i];
    }
    return s;
}

void loop() {};

void setup() {
  // initialize serial:
  Serial.begin(9600);

    int ORIGINAL_MESSAGE[] = {0, 1, 0, 1, 0, 1, 1};
    int size = sizeof(ORIGINAL_MESSAGE) / sizeof(ORIGINAL_MESSAGE[0]);
    int ENCODED_MESSAGE[2][size];
    
    // Initialize ENCODED_MESSAGE with zeros
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < size; ++j) {
            ENCODED_MESSAGE[i][j] = 0;
        }
    }

    int generator[2] = {0, 0};

    for (int i = 0; i < size; ++i) {
        int sum_7 = sum(generator, 2) + ORIGINAL_MESSAGE[i];
        int sum_5 = generator[1] + ORIGINAL_MESSAGE[i];

        // Set encoded bytes in the message
        ENCODED_MESSAGE[0][i] = sum_7 % 2;
        ENCODED_MESSAGE[1][i] = sum_5 % 2;

        //Propagate generator to next step
        generator[0] = ORIGINAL_MESSAGE[i];
        generator[1] = generator[0];
    }

    // Print ENCODED_MESSAGE to verify
    Serial.print("ENCODED_MESSAGE:\n");
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < size; ++j) {
          Serial.print(ENCODED_MESSAGE[i][j]);
          Serial.print(" ");
      }
      Serial.print("\n");
    }
        
    int SENT_MESSAGE[size*2];
    int k = 0;
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < 2; ++i) {
            SENT_MESSAGE[k] = ENCODED_MESSAGE[i][j];
            k++;
    }}

    Serial.print("1x14 array: ");
    for (int i = 0; i < size*2; ++i) {
        Serial.print(SENT_MESSAGE[i]);
        Serial.print(" ");
    }
    Serial.print("\n");
  }
