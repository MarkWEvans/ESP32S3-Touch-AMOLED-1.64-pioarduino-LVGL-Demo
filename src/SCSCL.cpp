/*
 * SCSCL.cpp
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2020.6.17
 * 作者: 
 */

#include "SCSCL.h"

SCSCL::SCSCL()
{
	End = 1;
}

SCSCL::SCSCL(u8 End):SCSerial(End)
{
}

SCSCL::SCSCL(u8 End, u8 Level):SCSerial(End, Level)
{
}

int SCSCL::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	u8 bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int SCSCL::WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	ACC = 0;
	u16 Time = 0;
	u8 bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int SCSCL::RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	u8 bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int SCSCL::CalibrationOfs(u8 ID){
	return -1;
}

void SCSCL::SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[])
{
    u8 offbuf[6*IDN];
    for(u8 i = 0; i<IDN; i++){
		u16 T, V;
		if(Time){
			T = Time[i];
		}else{
			T = 0;
		}
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
        Host2SCS(offbuf+i*6+0, offbuf+i*6+1, Position[i]);
        Host2SCS(offbuf+i*6+2, offbuf+i*6+3, T);
        Host2SCS(offbuf+i*6+4, offbuf+i*6+5, V);
    }
    syncWrite(ID, IDN, SCSCL_GOAL_POSITION_L, offbuf, 6);
}

int SCSCL::PWMMode(u8 ID)
{
	u8 bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);	
}

int SCSCL::WritePWM(u8 ID, s16 pwmOut)
{
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<10);
	}
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, pwmOut);
	
	return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}

int SCSCL::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable);
}

int SCSCL::unLockEprom(u8 ID)
{
	return writeByte(ID, SCSCL_LOCK, 0);
}

int SCSCL::LockEprom(u8 ID)
{
	return writeByte(ID, SCSCL_LOCK, 1);
}

int SCSCL::FeedBack(int ID)
{
	int nLen = Read(ID, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		Err = 1;
		return -1;
	}
	Err = 0;
	return nLen;
}
	
int SCSCL::ReadPos(int ID)
{
	int Pos = -1;
	if(ID==-1){
		Pos = Mem[SCSCL_PRESENT_POSITION_L-SCSCL_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= Mem[SCSCL_PRESENT_POSITION_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
		if(Pos==-1){
			Err = 1;
		}
	}
	return Pos;
}

int SCSCL::ReadSpeed(int ID)
{
	int Speed = -1;
	if(ID==-1){
		Speed = Mem[SCSCL_PRESENT_SPEED_L-SCSCL_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= Mem[SCSCL_PRESENT_SPEED_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
		if(Speed==-1){
			Err = 1;
			return -1;
		}
	}
	if(!Err && (Speed&(1<<15))){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

int SCSCL::ReadLoad(int ID)
{
	int Load = -1;
	if(ID==-1){
		Load = Mem[SCSCL_PRESENT_LOAD_L-SCSCL_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= Mem[SCSCL_PRESENT_LOAD_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
		if(Load==-1){
			Err = 1;
		}
	}
	if(!Err && (Load&(1<<10))){
		Load = -(Load&~(1<<10));
	}	
	return Load;
}

int SCSCL::ReadVoltage(int ID)
{
	int Voltage = -1;
	if(ID==-1){
		Voltage = Mem[SCSCL_PRESENT_VOLTAGE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
		if(Voltage==-1){
			Err = 1;
		}
	}
	return Voltage;
}

int SCSCL::ReadTemper(int ID)
{
	int Temper = -1;
	if(ID==-1){
		Temper = Mem[SCSCL_PRESENT_TEMPERATURE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
		if(Temper==-1){
			Err = 1;
		}
	}
	return Temper;
}

int SCSCL::ReadMove(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = Mem[SCSCL_MOVING-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Move = readByte(ID, SCSCL_MOVING);
		if(Move==-1){
			Err = 1;
		}
	}
	return Move;
}

int SCSCL::ReadMode(int ID)
{
	int ValueRead = -1;
	ValueRead = readWord(ID, SCSCL_MIN_ANGLE_LIMIT_L);
	if(ValueRead == 0){
		return 3;
	}
	else if(ValueRead > 0){
		return 0;
	}
	// int Mode = -1;
	// if(ID==-1){
	// 	Mode = Mem[SMS_STS_MODE-SMS_STS_PRESENT_POSITION_L];	
	// }else{
	// 	Err = 0;
	// 	Mode = readByte(ID, SMS_STS_MODE);
	// 	if(Mode==-1){
	// 		Err = 1;
	// 	}
	// }
	return ValueRead;
}

int SCSCL::ReadInfoValue(int ID, int AddInput)
{
	int ValueRead = -1;
	ValueRead = readWord(ID, AddInput);
	return ValueRead;
}

int SCSCL::ReadCurrent(int ID)
{
	int Current = -1;
	if(ID==-1){
		Current = Mem[SCSCL_PRESENT_CURRENT_L-SCSCL_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
		if(Current==-1){
			Err = 1;
			return -1;
		}
	}
	if(!Err && (Current&(1<<15))){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}

// Returns status.
int SCSCL::ReadStatus(int ID) {
	int statusByte = -1;
	if(ID==-1){
		statusByte = Mem[SCSCL_SERVO_STATUS-SCSCL_PRESENT_POSITION_L];
	} else{
		statusByte = readByte(ID, SCSCL_SERVO_STATUS);
	}
	return statusByte;
}

int SCSCL::ReadTorqueEnable(int id) {
	int torqueEnable = -1;
	Err = 0;
	torqueEnable = readByte(id, SCSCL_TORQUE_ENABLE);
	if(torqueEnable==-1){
		Err = 1;
	}
	return torqueEnable;
  }

   // Read maximum torque.
  int SCSCL::readMaxTorque(int id) {
    return readWord(id, SCSCL_MAX_TORQUE_LIMIT);
  }

  // Read overload torque (0~100%)
  // The maximum torque threshold for initiating the overload protection time countdown. 
  // For example, if set to 80, it represents 80% of the maximum torque.
  int SCSCL::readOverloadTorque(int id) {
    return readByte(id, SCSCL_OVERLOAD_TORQUE);
  }

  // Read maximum temperature limit.
  int SCSCL::readMaxTemperature(int id) {
    return readByte(id, SCSCL_MAX_TEMP_LIMIT);
  }

  // Read protection torque.
  int SCSCL::readProtectionTorque(int id) {
    return readByte(id, SCSCL_PROTECTION_TORQUE);
  }

  // Read protection time in milliseconds.
  int SCSCL::readProtectionTime(int id) {
    // Each unit corresponds to 40ms. Multiply accordingly.
    return readByte(id, SCSCL_PROTECTION_TIME) * 40;
  }


   // Write maximum torque, with optional EEPROM write.
   void SCSCL::writeMaxTorque(int id, int value, bool eeprom) {
    if (eeprom) {
      unLockEprom(id);
    }
    writeWord(id, SCSCL_MAX_TORQUE_LIMIT, value);
    if (eeprom) {
      LockEprom(id);  // Lock EPROM after writing.
    }
  }
  
  // Write maximum torque, with optional EEPROM write.
  void SCSCL::writeOverloadTorque(int id, int value, bool eeprom) {
    if (eeprom) {
      unLockEprom(id);
    }
    writeByte(id, SCSCL_OVERLOAD_TORQUE, value);
    if (eeprom) {
      LockEprom(id);  // Lock EPROM after writing.
    }
  }

   // Write maximum torque, with optional EEPROM write.
   void SCSCL::writeMaxTemperature(int id, int value, bool eeprom) {
    if (eeprom) {
      unLockEprom(id);
    }
    writeByte(id, SCSCL_MAX_TEMP_LIMIT, value);
    if (eeprom) {
      LockEprom(id);  // Lock EPROM after writing.
    }
  }

   // Write maximum torque, with optional EEPROM write.
   void SCSCL::writeProtectionTorque(int id, int value, bool eeprom) {
    if (eeprom) {
      unLockEprom(id);
    }
    writeByte(id, SCSCL_PROTECTION_TORQUE, value);
    if (eeprom) {
      LockEprom(id);  // Lock EPROM after writing.
    }
  }

   // Write maximum torque, with optional EEPROM write.
   void SCSCL::writeProtectionTime(int id, int value, bool eeprom) {
    if (eeprom) {
      unLockEprom(id);
    }
    writeByte(id, SCSCL_PROTECTION_TIME, value);
    if (eeprom) {
      LockEprom(id);  // Lock EPROM after writing.
    }
  }