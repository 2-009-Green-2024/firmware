// Placeholder sum function (to be defined properly as per your logic)
int sum(int arr[], int size) {
    int s = 0;
    for (int i = 0; i < size; ++i) {
        s += arr[i];
    }
    return s;
}


// Input is UnderwaterMessage
// Which has fields id, msg, data (id + msg concatenated)
// Ouput is (for 16-bit message), a 32-bit array of 1s and 0s
// Then, we will loop through this array and send tones based on it
// Example use:
// int[32] outputData;
UnderwaterMessage msg;
msg.id = 3;
// encode(msg, &outputData);
// Now, outputData has our encoded message in it
void encode(UnderwaterMessage inMessage, int *outputData) {
    // ORIGINAL_MESSAGE.data is your data array

    uint16_t ORIGINAL_MESSAGE = inMessage.data;
    
    int size = sizeof(ORIGINAL_MESSAGE);
    int ENCODED_MESSAGE[2][size];
    
    // Initialize ENCODED_MESSAGE with zeros
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < size; ++j) {
            ENCODED_MESSAGE[i][j] = 0;
        }
    }

    int generator[2] = {0, 0};

    for (int i = 0; i < size; ++i) {
        int sum_7 = ORIGINAL_MESSAGE[i] + generator[0] + generator[1];
        int sum_5 = ORIGINAL_MESSAGE[i] + generator[1];

        // Set encoded bytes in the message
        ENCODED_MESSAGE[0][i] = sum_7 % 2;
        ENCODED_MESSAGE[1][i] = sum_5 % 2;

        //Propagate generator to next step
        generator[1] = generator[0];
        generator[0] = ORIGINAL_MESSAGE[i];
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

    return 
  }
   

UnderwaterMessage decode(int* received) {
    // 16 bits long
    // First 8 bits - message
    // Last 8 bits - ID
}