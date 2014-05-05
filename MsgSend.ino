byte frameHeader[] = {
  0x7E, 0, 0}; // Start delimiter, count MSB, count LSB
byte tXframeDataHeader[] = {
  0x01, 0, 0, 0, 0x01}; // API id, Frame id, dest MSB, dest LSB, Options
 
/*********************************************************
 *
 * sendTXFrame()
 *
 *     Send a TX frame to the specified destination.
 *     The data wlll be int the rfData.  Prepend
 *     the API id, Frame Id, Destination Address, and 
 *     Options bytes
 *     rfDataLenth is always the offset AFTER the last
 *     array value.
 *
 *********************************************************/
void sendTXFrame(int dest, byte rfData[], int rfDataLength) {  
  tXframeDataHeader[2] = dest/256;    // MSB
  tXframeDataHeader[3] = dest & 0xFF; // LSB
  sendFrame(tXframeDataHeader, 5, rfData, rfDataLength);
}


/*********************************************************
 *
 * sendFrame()
 *
 *     Send the API frame with the given Data Header
 *     and Data. Prepend the Start Delimiter (0cFE)
 *     and the Length bytes.  Append the Checksum
 *     before sending.
 *
 *********************************************************/
void sendFrame(byte cmdDataHeader[], int cmdDataHeaderLength, byte cmdData[], int cmdDataLength) {
  unsigned int sum = 0;

  // Set the length in bytes 2 & 3.
  frameHeader[1] = 0;
  frameHeader[2] = cmdDataHeaderLength + cmdDataLength;

  // Compute the checksum.
  for (int i = 0; i < cmdDataHeaderLength; i++) {
    sum += cmdDataHeader[i];
  }
  for (int i = 0; i < cmdDataLength; i++) {
    sum += cmdData[i];
  }

  SERIAL.write(frameHeader, 3);
  if (cmdDataHeaderLength > 0) {
    SERIAL.write(cmdDataHeader, cmdDataHeaderLength);
  }
  if (cmdDataLength > 0) {
    SERIAL.write(cmdData, cmdDataLength);
  }
  SERIAL.write(0xFF - sum);
}


void set2Byte(byte array[], int index, int value) {
  array[index + 1] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index] = (byte) value;
}
void set4Byte(byte array[], int index, int value) {
  array[index + 3] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index + 2] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index + 1] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index] = (byte) value;
}



/*********************************************************
 *
 * dump()
 *
 *     If CMD_STATE_DUMP & TP_STATE_DUMPING is true, we are
 *     collecting data.  If only TP_STATE_DUMPING is true,
 *     we are dumping it to the PC.
 *     
 *
 *********************************************************/
void dump() {
  if (    isBitClear(cmdState, CMD_STATE_DUMP) 
      && (isBitClear(tpState, TP_STATE_DUMPING))) {
        return;
  }
      
    // Are we are collecting data.
    if ( isBitSet(cmdState, CMD_STATE_DUMP) 
        && (isBitSet(tpState, TP_STATE_DUMPING))) {
      
  }
}

