//// ----------------------------------------------------------------------------------
////  ecana.c
//// ----------------------------------------------------------------------------------

#include "../../F2837xD_common/include/F2837xD_Project.h"
#include "../../F2837xD_common/inc/hw_memmap.h"
#include "ecan.h"
#include "platform.h"
//// --------------------------------RegsBox-------------------------------------------
MBOXS mboxs_b;
tCANMsgObject *mboxb[32] = {
		&mboxs_b.MBOX0,
		&mboxs_b.MBOX1,
		&mboxs_b.MBOX2,
		&mboxs_b.MBOX3,
		&mboxs_b.MBOX4,
		&mboxs_b.MBOX5,
		&mboxs_b.MBOX6,
		&mboxs_b.MBOX7,
		&mboxs_b.MBOX8,
		&mboxs_b.MBOX9,
		&mboxs_b.MBOX10,
		&mboxs_b.MBOX11,
		&mboxs_b.MBOX12,
		&mboxs_b.MBOX13,
		&mboxs_b.MBOX14,
		&mboxs_b.MBOX15,
		&mboxs_b.MBOX16,
		&mboxs_b.MBOX17,
		&mboxs_b.MBOX18,
		&mboxs_b.MBOX19,
		&mboxs_b.MBOX20,
		&mboxs_b.MBOX21,
		&mboxs_b.MBOX22,
		&mboxs_b.MBOX23,
		&mboxs_b.MBOX24,
		&mboxs_b.MBOX25,
		&mboxs_b.MBOX26,
		&mboxs_b.MBOX27,
		&mboxs_b.MBOX28,
		&mboxs_b.MBOX29,
		&mboxs_b.MBOX30,
		&mboxs_b.MBOX31,
};
static void S_Monitor(void);
static void R_Monitor(void);

static unsigned char s_msgdata[8] = {0,0,0,0,0,0,0,0};
static int unsendcount = 0;
static int sendlong = 0;
static int sendbegin = 0;
static UINT16 sendtime = 0;
static UINT16 sendtimeout = 0;
static bool sendbusy = false;

static unsigned char r_msgdata[8] = {0,0,0,0,0,0,0,0};
static canmsg* recv_msg = NULL;
static int recv_long = 0;
static int recvbegin = 0;
static int recv_time = 0;
static int recv_timeout = 0;
static int recv_count = 0;
static bool recvbusy = false;
///// ----------------------------------config--------------------------------------
static CanRetEnum ConfigEcan(int sdo, int pdo, Uint32 bps, int dbo) {
	if((sdo + pdo) > 16)
		return eCanchannelUnenough;
	int i = 0;
	Uint32 mm = 0x18000000;				//600
	Uint32 nn = 0x16000000;				//580
	for(i = 0;i < sdo;i++){
		mm += 0x40000;
		nn += 0x40000;
		mboxb[i]->ui32MsgID = mm;
		mboxb[i]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxb[i]->ui32MsgIDMask = 0;
		mboxb[i]->ui32MsgLen = 8;
		mboxb[i + 16]->ui32MsgID = nn;
		mboxb[i + 16]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxb[i + 16]->ui32MsgIDMask = 0;
		mboxb[i + 16]->ui32MsgLen = 8;
	}
	mm = 0x8000000;				//200
	nn = 0x6000000;				//180
	for(i = 0;i < pdo;i++){
		mm += 0x40000;
		nn += 0x40000;
		mboxb[i + sdo]->ui32MsgID = mm;
		mboxb[i + sdo]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxb[i + sdo]->ui32MsgIDMask = 0;
		mboxb[i + sdo]->ui32MsgLen = 8;
		mboxb[i + sdo + 16]->ui32MsgID = nn;
		mboxb[i + sdo + 16]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxb[i + sdo + 16]->ui32MsgIDMask = 0;
		mboxb[i + sdo + 16]->ui32MsgLen = 8;
	}
//	if(pdo)
//		mboxb[sdo + pdo - 1]->ui32MsgID = 0x2000000;	//80

	EALLOW;
	CpuSysRegs.PCLKCR10.bit.CAN_B = 1;
	EDIS;

	CANInit(CANB_BASE);
	CANClkSourceSelect(CANB_BASE, 0);
	CANBitRateSet(CANB_BASE, 100000000, bps);
	CANEnable(CANB_BASE);
	return eCanConfigSucceed;
}
///// -------------------------------------sendmsg---------------------------------------------
static CanRetEnum SendEcanMsg(unsigned short mark, canmsg* msg, int begin, int msglong, unsigned short timeout){
	if(mark <= 0 || mark > 0xFFFF || begin < 0 || begin >= 16 || msg == NULL || msglong == 0)
		return eCanParameterInvalid;
	if(sendbusy)
		return eCanPortOccupied;
	else
	{
		int i = 0;
		sendbegin = begin;
		sendtimeout = timeout;
		sendlong = msglong;
		sendbusy = true;
 		for(;sendbegin < msglong;sendbegin++){
			if(mark & (0x1 << sendbegin)){
				for(i = 0;i < 4;i++){
					s_msgdata[i] = (msg[sendbegin - begin].cmd >> (8 * i)) & 0xFF;
					s_msgdata[i + 4] = (msg[sendbegin - begin].data >> (8 * i)) & 0xFF;
				}
				mboxb[sendbegin]->pucMsgData = s_msgdata;
				CANMessageSet(CANB_BASE, sendbegin + 1, mboxb[sendbegin], MSG_OBJ_TYPE_TX);
			}
		}
		CreateSysTimerInterface()->Start(S_Monitor, 1U);
		return LoadDataSucceed;
	}
}
static void S_Monitor(void){
	sendtime++;
	int i;
	uint32_t a;
	a = CANStatusGet(CANB_BASE, CAN_STS_TXREQUEST);
	if(!a){
		CreateSysTimerInterface()->Stop(S_Monitor);
		is_b_s_timeout = false;
		canb_send += sendlong;
		current_b_send = sendlong;
		sendbusy = false;
		sendtime = 0;
	}
	else if(sendtime >= sendtimeout){
		CreateSysTimerInterface()->Stop(S_Monitor);
		for(i = sendbegin;i < sendlong;i++){
			if(a & (1ul << i))
				unsendcount++;
		}
		is_b_s_timeout = true;
		canb_send += sendlong - unsendcount;
		current_b_send = sendlong - unsendcount;
		sendbusy = false;
		unsendcount = 0;
		sendtime = 0;
	}
}
///// -------------------------------------recvmsg------------------------------------
static CanRetEnum RecvEcanMsg(unsigned short mark, canmsg* msg, int begin, int msglong, unsigned short timeout) {
	if(mark <= 0 || mark > 0xFFFF || msglong == 0)
		return eCanParameterInvalid;
	if(recvbusy)
		return eCanPortOccupied;
	else
	{
		recvbegin = begin;
		recv_timeout = timeout;
		recv_long = msglong;
		recv_msg = msg;
		recvbusy = true;
		for(;begin < msglong;begin++){
			if(mark & (1u << recvbegin)){
				mboxb[begin]->pucMsgData = r_msgdata;
				CANMessageSet(CANB_BASE, begin + 1, mboxb[begin], MSG_OBJ_TYPE_RX);
			}
		}
		CreateSysTimerInterface()->Start(R_Monitor, 1U);
		return eCanSucceed;
	}
}

static void R_Monitor(void){
	recv_time++;
	int i = 0,j = 0;
	if(CANStatusGet(CANB_BASE, CAN_STS_NEWDAT)){
		for(i = recvbegin;i < recv_long;i++){
			if(CANStatusGet(CANB_BASE, CAN_STS_NEWDAT) & (0x1u << i)){
				CANMessageGet(CANB_BASE, i + 1, mboxb[i], true);
				for(j = 3;j >= 0;j--){
					recv_msg[i - recvbegin].cmd <<= 8;
					recv_msg[i - recvbegin].data <<= 8;
					recv_msg[i - recvbegin].cmd += r_msgdata[j];
					recv_msg[i - recvbegin].data += r_msgdata[j + 4];
				}
				recv_count++;
			}
		}
	}
	if(recv_time >= recv_timeout || recv_count >= recv_long){
		CreateSysTimerInterface()->Stop(R_Monitor);
		if(recv_count >= recv_long)
			is_b_r_timeout = false;
		else if(recv_time >= recv_timeout)
			is_b_r_timeout = true;
		canb_read += recv_count;
		current_b_read = recv_count;
		recvbusy = false;
		recv_count = 0;
		recv_time = 0;
	}
}
///// ---------------------------------interface--------------------------------------
const EcanInterface* EcanbInterface_b(void){
	static const EcanInterface ecan = {
			ConfigEcan,
			RecvEcanMsg,
			SendEcanMsg
	};
	return &ecan;
}
///// ------------------------------------e-------------------------------------------
///// ------------------------------------n-------------------------------------------
///// ------------------------------------d-------------------------------------------
