#include <SCServo.h>

SCServo::SCServo (UART_HandleTypeDef *huart) : huart_(huart)
{
}

void SCServo::Printf(u8 reg)
{
    HAL_UART_Transmit(huart_, &reg, 1, 10);
    uint8_t data;
    HAL_UART_Receive(huart_, &data, 1, 10); // Cause we receive sent bytes (single wire)
}

void SCServo::fflushRevBuf(void)
{
	uint8_t data;
	while(HAL_UART_Receive(huart_, &data, 1, 0)==HAL_OK);
    return;
}

int SCServo::EnableTorque(u8 ID, u8 Enable, u8 ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_TORQUE_ENABLE);
    Printf(Enable);
    Printf((~(ID + messageLength + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
    if(ID !=  0xfe && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WritePos(u8 ID, int position, int velocity, u8 ReturnLevel)
{
    int messageLength = 7;
    u8 posL = position>>8;
    u8 posH = position&0xff;
    u8 velL = velocity>>8;
    u8 velH = velocity&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(posL);
    Printf(posH);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::RegWritePos(u8 ID, int position, int velocity, u8 ReturnLevel)
{
    int messageLength = 7;
    u8 posL = position>>8;
    u8 posH = position&0xff;
    u8 velL = velocity>>8;
    u8 velH = velocity&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_REG_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(posL);
    Printf(posH);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_REG_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

void SCServo::RegWriteAction()
{
    int messageLength = 2;
    u8 ID = 16;
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_ACTION);
    Printf((~(ID + messageLength + INST_ACTION))&0xFF);
}

int SCServo::ReadBuf(u16 len, u8 *buf)
{
	int ret = HAL_UART_Receive(huart_, buf, len, 1000);
    if(ret==HAL_OK) {
    	return len;
    }
    return -1;

}

int SCServo::ReadPos(u8 ID)
{
    u8 buf[8] = {0};
    int size;
    int pos=0;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_POSITION_L);
    Printf(2);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_POSITION_L + 2))&0xFF);
    size = ReadBuf(8, buf);
    if(size<8)
        return -1;
    pos = buf[5];
    pos <<= 8;
    pos |= buf[6];
    return pos;
}

void SCServo::SyncWritePos(u8 ID[], u8 IDN, int position, int velocity)
{
    int messageLength = 5*IDN+4;
    u8 Sum = 0;
    u8 posL = position>>8;
    u8 posH = position&0xff;

    u8 velL = velocity>>8;
    u8 velH = velocity&0xff;

    Printf(startByte);
    Printf(startByte);
    Printf(16);
    Printf(messageLength);
    Printf(INST_SYNC_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(4);

    Sum = 16 + messageLength + INST_SYNC_WRITE + P_GOAL_POSITION_L + 4;
    int i;
    for(i=0; i<IDN; i++){
        Printf(ID[i]);
        Printf(posL);
        Printf(posH);
        Printf(velL);
        Printf(velH);
        Sum += ID[i] + posL + posH + velL + velH;
    }
    Printf((~Sum)&0xFF);
}

int SCServo::WriteID(u8 oldID, u8 newID, u8 ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(oldID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_ID);
    Printf(newID);
    Printf((~(oldID + messageLength + INST_WRITE + newID + P_ID))&0xFF);
    if(oldID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteLimitAngle(u8 ID, int MinAngel, int MaxAngle, u8 ReturnLevel)
{
    int messageLength = 7;
    u8 MinAL = MinAngel>>8;
    u8 MinAH = MinAngel&0xff;
    u8 MaxAL = MaxAngle>>8;
    u8 MaxAH = MaxAngle&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_MIN_ANGLE_LIMIT_L);
    Printf(MinAL);
    Printf(MinAH);
    Printf(MaxAL);
    Printf(MaxAH);
    Printf((~(ID + messageLength + INST_WRITE + P_MIN_ANGLE_LIMIT_L + MinAL + MinAH + MaxAL + MaxAH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteLimitTroque(u8 ID, int MaxTroque, u8 ReturnLevel)
{
    int messageLength = 5;
    u8 MaxTL = MaxTroque>>8;
    u8 MaxTH = MaxTroque&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_MAX_TORQUE_L);
    Printf(MaxTL);
    Printf(MaxTH);

    Printf((~(ID + messageLength + INST_WRITE + P_MAX_TORQUE_L + MaxTL + MaxTH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WritePunch(u8 ID, int Punch, u8 ReturnLevel)
{
    int messageLength = 5;
    u8 PunchL = Punch>>8;
    u8 PunchH = Punch&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_PUNCH_L);
    Printf(PunchL);
    Printf(PunchH);

    Printf((~(ID + messageLength + INST_WRITE + P_PUNCH_L + PunchL + PunchH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteBaund(u8 ID, u8 Baund, u8 ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_BAUD_RATE);
    Printf(Baund);

    Printf((~(ID + messageLength + INST_WRITE + P_BAUD_RATE + Baund))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteDeadBand(u8 ID, u8 CWDB, u8 CCWDB, u8 ReturnLevel)
{
    int messageLength = 5;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_CW_DEAD);
    Printf(CWDB);
    Printf(CCWDB);

    Printf((~(ID + messageLength + INST_WRITE + P_CW_DEAD + CWDB + CCWDB))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteIMax(u8 ID, int IMax, u8 ReturnLevel)
{
    int messageLength = 5;

    u8 velL = IMax>>8;
    u8 velH = IMax&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_IMAX_L);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_IMAX_L + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}


int SCServo::LockEprom(u8 ID, u8 Enable, u8 ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_LOCK);
    Printf(Enable);

    Printf((~(ID + messageLength + INST_WRITE + P_LOCK + Enable))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WritePID(u8 ID, u8 P, u8 I, u8 D, u8 ReturnLevel)
{
    int messageLength = 6;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_COMPLIANCE_P);
    Printf(P);
    Printf(D);
    Printf(I);

    Printf((~(ID + messageLength + INST_WRITE + P_COMPLIANCE_P + P + D + I))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServo::WriteSpe(u8 ID, int velocity, u8 ReturnLevel)
{
    int messageLength = 5;

    int vel = velocity;
    if(velocity<0){
        vel = -velocity;
        vel |= (1<<10);
    }

    u8 velL =  vel>>8;
    u8 velH =  vel&0xff;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_GOAL_SPEED_L);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_GOAL_SPEED_L + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}


int SCServo::ReadVoltage(u8 ID)
{
    u8 buf[7] = {0};
    u8 size;
    int vol;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_VOLTAGE);
    Printf(1);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_VOLTAGE + 1))&0xFF);
    size = ReadBuf(7, buf);
    if(size<7)
        return -1;
    vol = buf[5];
    return vol;
}

int SCServo::ReadTemper(u8 ID)
{
    u8 buf[7] = {0};
    u8 size;
    int temper;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_TEMPERATURE);
    Printf(1);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_TEMPERATURE + 1))&0xFF);
    size = ReadBuf(7, buf);
    if(size<7)
        return -1;
    temper = buf[5];
    return temper;
}

void SCServo::RotateClockwise(){
    EnableTorque(16, 1);
    HAL_Delay(200);
    WritePos(16, 400, 500);
}

void SCServo::RotateCounterClockwise(){
    EnableTorque(16, 1);
    HAL_Delay(200);
    WritePos(16, 750, 500);
}
