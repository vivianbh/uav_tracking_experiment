#include <string.h>
#include "Seeker.hpp"

/**************************************************************************************************/
/* uav data receieved                                                                          */
/**************************************************************************************************/
uint8_t  uavRcvBuff[64];
uint8_t  uavPacketReceived = 0;
uint16_t uavPacketCnt = 0;
uint16_t uavPacketErrorCnt = 0;
uint16_t uavPacketFreq = 0;
uavData_t uavData;

void uavParser(uint8_t Rxbyte){

    static uint8_t  state = 0;
    static uint8_t  msgLen = 0;
    static uint8_t  msgId = 0;
    static uint8_t  chkSumRcv = 0;
    static uint16_t chkSum = 0;
    static uint16_t count = 0;

    switch( state ){
        // $DAT
        case 0:
            if( Rxbyte == '$' )
                state = 1;
            else
                state = 0;
            break;

        case 1:
            if( Rxbyte == 'D' )
                state = 2;
            else
                state = 0;
            break;

        case 2:
            if( Rxbyte == 'A' )
                state = 3;
            else
                state = 0;
            break;

        case 3:
            if( Rxbyte == 'T' )
                state = 4;
            else
                state = 0;
            break;

        // [MSG LEN]
        case 4:
            msgLen = Rxbyte;
            if( msgLen == (sizeof(uavData_t)+1) ){
                chkSum = Rxbyte;
                state = 5;
            }
            else
                state = 0;
            break;

        // [MSG ID]
        case 5:
            msgId = Rxbyte;
            if( msgId == 0x02 ){
                chkSum += Rxbyte;
                state = 6;
                count = 0;
            }
            else
                state = 0;
            break;

        // [DATA]
        case 6:
            if( count < sizeof(uavRcvBuff) ){
                uavRcvBuff[count] = Rxbyte;
                chkSum += Rxbyte;
                count++;
                if( count >= sizeof(uavData_t) ){
                    state = 7;
                }
            }
            else
                state = 0;
            break;

        // [CHKSUM]
        case 7:
            chkSumRcv = Rxbyte;
            state = 8;
            break;

        // [END]
        case 8:
            if( ((uint8_t)(chkSum&0x00FF) == chkSumRcv) && (Rxbyte == 0x0A) ) {
                uavPacketCnt++;
                uavPacketReceived = 1;
            }
            else {
                uavPacketErrorCnt++;
            }
            state = 0;
            break;
    }
}

void getUavData(void){
    if( uavPacketReceived ){
        memcpy((uint8_t*)&uavData,  (uint8_t*)&uavRcvBuff, sizeof(uavData_t));
        uavPacketReceived = 0;
    }
}

/**************************************************************************************************/
/* uav command send, call every 20ms(default).                                                 */
/**************************************************************************************************/
uint8_t uavCmd_Txbuff[(sizeof(uav_CMD_t)+8)];
uav_CMD_t uavCmd = {0, 0, 0, 0, 420, 420, 0, 3746, 10, 0};
uint16_t uavCmd_checkSum = 0;

uav_CMD_t currentUavCmd(void){
    return uavCmd;
}

void updateUavCmd(uav_CMD_t cmd){
    uavCmd = cmd;
}

void sendUavCommand(void){
    uavCmd_Txbuff[0] = '$';
    uavCmd_Txbuff[1] = 'D';
    uavCmd_Txbuff[2] = 'A';
    uavCmd_Txbuff[3] = 'T';
    uavCmd_Txbuff[4] = sizeof(uav_CMD_t) + 1;
    uavCmd_Txbuff[5] = 0x82;

    memcpy((uint8_t*)&uavCmd_Txbuff[6], (uint8_t*)&uavCmd, sizeof(uav_CMD_t));

    uavCmd_checkSum = 0;
    for( uint8_t index = 4 ; index < (sizeof(uav_CMD_t)+6) ; index++){
        uavCmd_checkSum += uavCmd_Txbuff[index];
    }

    uavCmd_Txbuff[(sizeof(uav_CMD_t)+6)] = (uint8_t)(uavCmd_checkSum&0x00FF);
    uavCmd_Txbuff[(sizeof(uav_CMD_t)+7)] = 0x0A;

    /* send seekerCmd_Txbuff out by user*/
    //UART_SendData((uint8_t*)&uavCmd_Txbuff, sizeof(uavCmd_Txbuff));
}

uint8_t* UART_SendData(){
    //printf("uavCmd_Txbuff: %p\n", &uavCmd_Txbuff);
    return uavCmd_Txbuff;
}

/**************************************************************************************************/
/* gcs data receieved                                                                          */
/**************************************************************************************************/
uint8_t  gcsRcvBuff[64];
uint8_t  gcsPacketReceived = 0;
uint16_t gcsPacketCnt = 0;
uint16_t gcsPacketErrorCnt = 0;
uint16_t gcsPacketFreq = 0;
uav_CMD_t gcsData;

void gcsParser(uint8_t Rxbyte){

    static uint8_t  state = 0;
    static uint8_t  msgLen = 0;
    static uint8_t  msgId = 0;
    static uint8_t  chkSumRcv = 0;
    static uint16_t chkSum = 0;
    static uint16_t count = 0;

    switch( state ){
        // $DAT
        case 0:
            if( Rxbyte == '$' )
                state = 1;
            else
                state = 0;
            break;

        case 1:
            if( Rxbyte == 'D' )
                state = 2;
            else
                state = 0;
            break;

        case 2:
            if( Rxbyte == 'A' )
                state = 3;
            else
                state = 0;
            break;

        case 3:
            if( Rxbyte == 'T' )
                state = 4;
            else
                state = 0;
            break;

        // [MSG LEN]
        case 4:
            msgLen = Rxbyte;
            if( msgLen == (sizeof(uav_CMD_t)+1) ){
                chkSum = Rxbyte;
                state = 5;
            }
            else
                state = 0;
            break;

        // [MSG ID]
        case 5:
            msgId = Rxbyte;
            if( msgId == 0x82 ){
                chkSum += Rxbyte;
                state = 6;
                count = 0;
            }
            else
                state = 0;
            break;

        // [DATA]
        case 6:
            if( count < sizeof(gcsRcvBuff) ){
                gcsRcvBuff[count] = Rxbyte;
                chkSum += Rxbyte;
                count++;
                if( count >= sizeof(uav_CMD_t) ){
                    state = 7;
                }
            }
            else
                state = 0;
            break;

        // [CHKSUM]
        case 7:
            chkSumRcv = Rxbyte;
            state = 8;
            break;

        // [END]
        case 8:
            if( ((uint8_t)(chkSum&0x00FF) == chkSumRcv) && (Rxbyte == 0x0A) ) {
                gcsPacketCnt++;
                gcsPacketReceived = 1;
            }
            else {
                gcsPacketErrorCnt++;
            }
            state = 0;
            break;
    }
}

void getGcsData(void){
    if( gcsPacketReceived ){
        cout << "yes" << endl;
        memcpy((uint8_t*)&gcsData,  (uint8_t*)&gcsRcvBuff, sizeof(uav_CMD_t));
        gcsPacketReceived = 0;
    }
}

uav_CMD_t currentGcsData(void){
    return gcsData;
}
