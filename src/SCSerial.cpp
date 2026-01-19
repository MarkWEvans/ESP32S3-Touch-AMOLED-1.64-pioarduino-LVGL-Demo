/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2019.4.27
 * 作者: 
 */


#include "SCSerial.h"
#include <stddef.h>

SCSerial::SCSerial()
{
	IOTimeOut = 10;
	pSerial = NULL;
	Level = 1;//除广播指令所有指令返回应答
	Error = 0;
}

SCSerial::SCSerial(u8 End)
{
	IOTimeOut = 10;
	pSerial = NULL;
	Level = 1;
	this->End = End;
	Error = 0;
}

SCSerial::SCSerial(u8 End, u8 Level)
{
	IOTimeOut = 10;
	pSerial = NULL;
	this->Level = Level;
	this->End = End;
	Error = 0;
}

void SCSerial::begin(HardwareSerial& serial, unsigned long timeout) {
    pSerial   = &serial;
    IOTimeOut = timeout;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;
	while(1){
		ComData = pSerial->read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if(Size>=nLen){
			break;
		}
		t_user = millis() - t_begin;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}

void SCSerial::rFlushSCS()
{
	while(pSerial->read()!=-1);
}

void SCSerial::wFlushSCS()
{
}

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SCSerial::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
u16 SCSerial::SCS2Host(u8 DataL, u8 DataH)
{
	u16 Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

void SCSerial::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	writeSCS(~CheckSum);
}

//普通写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCSerial::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCSerial::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写执行指令
//舵机ID
int SCSerial::RegWriteAction(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCSerial::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

int SCSerial::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

int SCSerial::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, wDat);
	rFlushSCS();
	writeBuf(ID, MemAddr, bBuf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

// Fixed by Mark on 2025-04-26
int SCSerial::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
    wFlushSCS();

    if(!checkHead())        return 0;
    u8 hdr[3];
    if(readSCS(hdr, 3)!=3)  return rFlushAndFail();
    u8 length    = hdr[1];
    u8 paramLen  = (length>=2 ? length-2 : 0);

    // read whatever they actually send
    if(paramLen > 0) {
      // only read as many bytes as will actually fit in nData
      u8 toCopy = (paramLen < nLen ? paramLen : nLen);
      if(readSCS(nData, toCopy) != toCopy)
        return rFlushAndFail();

      // if the servo sent more than we have buffer for, discard the rest
      int extra = paramLen - toCopy;
      static u8 dummy[32];
      while(extra > 0) {
        u8 chunk = (extra < (int)sizeof(dummy) ? extra : sizeof(dummy));
        int got = readSCS(dummy, chunk);
        if(got <= 0) break;
        extra -= got;
      }
    }

    u8 chk;
    if(readSCS(&chk, 1)!=1) return rFlushAndFail();

    // verify checksum…
    // if checksum fails: return rFlushAndFail();

    // if they sent fewer than YOU wanted, still fine to return 0 now:
    if(paramLen != nLen)   return 0;

    Error = hdr[2];
    return paramLen;
}

inline int SCSerial::rFlushAndFail() {
  // assume max packet length is, say, 32 bytes
  u8 dummy[32];
  readSCS(dummy, sizeof(dummy));    // blocks until timeout or full
  return 0;
}


//读1字节，超时返回-1
int SCSerial::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//读2字节，超时返回-1
int SCSerial::readWord(u8 ID, u8 MemAddr)
{	
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

//Ping指令，返回舵机ID，超时返回-1
int	SCSerial::Ping(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	Error = 0;
	if(!checkHead()){
		return -1;
	}
	u8 bBuf[4];
	if(readSCS(bBuf, 4)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;			
	}
	Error = bBuf[2];
	return bBuf[0];
}

int SCSerial::checkHead()
{
	u8 bDat;
	u8 bBuf[2] = {0, 0};
	u8 Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){
			return 0;
		}
	}
	return 1;
}

int	SCSerial::Ack(u8 ID)
{
	Error = 0;
	if(ID!=0xfe && Level){
		if(!checkHead()){
			return 0;
		}
		u8 bBuf[4];
		if(readSCS(bBuf, 4)!=4){
			return 0;
		}
		if(bBuf[0]!=ID){
			return 0;
		}
		if(bBuf[1]!=2){
			return 0;
		}
		u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			return 0;			
		}
		Error = bBuf[2];
	}
	return 1;
}

int	SCSerial::syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen)
{
	syncReadRxPacketLen = nLen;
	u8 checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	u8 i;
	writeSCS(0xff);
	writeSCS(0xff);
	writeSCS(0xfe);
	writeSCS(IDN+4);
	writeSCS(INST_SYNC_READ);
	writeSCS(MemAddr);
	writeSCS(nLen);
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeSCS(checkSum);
	return nLen;
}

int SCSerial::syncReadPacketRx(u8 ID, u8 *nDat)
{
    syncReadRxPacket      = nDat;
    syncReadRxPacketIndex = 0;

    // 1) header
    if(!checkHead()) return 0;

    // 2) read ID, length, error
    u8 header[3];
    if(readSCS(header, 3) != 3) return 0;
    u8 respID    = header[0];
    u8 length    = header[1];      // = #params + 2
    u8 errStatus = header[2];

    if(respID != ID) return 0;
    Error = errStatus;

    // 3) actual parameter count:
    u8 paramLen = (length >= 2 ? length - 2 : 0);

    // 4) read parameters
    if(paramLen > 0) {
      int got = readSCS(nDat, paramLen);
      if(got != paramLen) {
        // rFlushSCS();
        return 0;
      }
    }

    // 5) read checksum
    u8 chk;
    if(readSCS(&chk, 1) != 1) return 0;

    // 6) verify checksum
    u16 sum = header[0] + header[1] + header[2];
    for(u8 i = 0; i < paramLen; i++) sum += nDat[i];
    if((u8)(~sum) != chk) return 0;

    // 7) if fewer bytes than expected, fail
    if(paramLen != syncReadRxPacketLen) return 0;

    return paramLen;
}

int SCSerial::syncReadRxPacketToByte()
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

int SCSerial::syncReadRxPacketToWrod(u8 negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}
