//// ----------------------------------------------------------------------------------
////  ecana.c
//// ----------------------------------------------------------------------------------

#include "../../F2837xD_common/include/F2837xD_Project.h"
#include "../../F2837xD_common/inc/hw_memmap.h"
#include "ecan.h"
#include "platform.h"
//// --------------------------------RegsBox-------------------------------------------
MBOXS mboxs_a;
tCANMsgObject *mboxa[32] = {
		&mboxs_a.MBOX0,
		&mboxs_a.MBOX1,
		&mboxs_a.MBOX2,
		&mboxs_a.MBOX3,
		&mboxs_a.MBOX4,
		&mboxs_a.MBOX5,
		&mboxs_a.MBOX6,
		&mboxs_a.MBOX7,
		&mboxs_a.MBOX8,
		&mboxs_a.MBOX9,
		&mboxs_a.MBOX10,
		&mboxs_a.MBOX11,
		&mboxs_a.MBOX12,
		&mboxs_a.MBOX13,
		&mboxs_a.MBOX14,
		&mboxs_a.MBOX15,
		&mboxs_a.MBOX16,
		&mboxs_a.MBOX17,
		&mboxs_a.MBOX18,
		&mboxs_a.MBOX19,
		&mboxs_a.MBOX20,
		&mboxs_a.MBOX21,
		&mboxs_a.MBOX22,
		&mboxs_a.MBOX23,
		&mboxs_a.MBOX24,
		&mboxs_a.MBOX25,
		&mboxs_a.MBOX26,
		&mboxs_a.MBOX27,
		&mboxs_a.MBOX28,
		&mboxs_a.MBOX29,
		&mboxs_a.MBOX30,
		&mboxs_a.MBOX31,
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
		mboxa[i]->ui32MsgID = mm;
		mboxa[i]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxa[i]->ui32MsgIDMask = 0;
		mboxa[i]->ui32MsgLen = 8;
		mboxa[i + 16]->ui32MsgID = nn;
		mboxa[i + 16]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxa[i + 16]->ui32MsgIDMask = 0;
		mboxa[i + 16]->ui32MsgLen = 8;
	}
	mm = 0x8000000;				//200
	nn = 0x6000000;				//180
	for(i = 0;i < pdo;i++){
		mm += 0x40000;
		nn += 0x40000;
		mboxa[i + sdo]->ui32MsgID = mm;
		mboxa[i + sdo]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxa[i + sdo]->ui32MsgIDMask = 0;
		mboxa[i + sdo]->ui32MsgLen = 8;
		mboxa[i + sdo + 16]->ui32MsgID = nn;
		mboxa[i + sdo + 16]->ui32Flags = MSG_OBJ_NO_FLAGS;
		mboxa[i + sdo + 16]->ui32MsgIDMask = 0;
		mboxa[i + sdo + 16]->ui32MsgLen = 8;
	}
//	if(pdo)
//		mboxa[sdo + pdo - 1]->ui32MsgID = 0x2000000;	//80

	EALLOW;
	CpuSysRegs.PCLKCR10.bit.CAN_A = 1;
	EDIS;

	// Initialize the CAN controller
	CANInit(CANA_BASE);

	// Setup CAN to be clocked off the M3/Master subsystem clock
	CANClkSourceSelect(CANA_BASE, 0);   /* 500kHz CAN-Clock */

	// Set up the bit rate for the CAN bus.  This function sets up the CAN
	// bus timing for a nominal configuration.  You can achieve more control
	// over the CAN bus timing by using the function CANBitTimingSet() instead
	// of this one, if needed.
	// In this example, the CAN bus is set to 500 kHz.  In the function below,
	// the call to SysCtlClockGet() is used to determine the clock rate that
	// is used for clocking the CAN peripheral.  This can be replaced with a
	// fixed value if you know the value of the system clock, saving the extra
	// function call.  For some parts, the CAN peripheral is clocked by a fixed
	// 8 MHz regardless of the system clock in which case the call to
	// SysCtlClockGet() should be replaced with 8000000.  Consult the data
	// sheet for more information about CAN peripheral clocking.
	CANBitRateSet(CANA_BASE, 100000000, bps);

	// Enable the CAN for operation.
	CANEnable(CANA_BASE);
	return eCanConfigSucceed;
}

///// -------------------------------------sendmsg------------------------------------
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
				mboxa[sendbegin]->pucMsgData = s_msgdata;
				CANMessageSet(CANA_BASE, sendbegin + 1, mboxa[sendbegin], MSG_OBJ_TYPE_TX);
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
	a = CANStatusGet(CANA_BASE, CAN_STS_TXREQUEST);
	if(!a){
		CreateSysTimerInterface()->Stop(S_Monitor);
		is_a_s_timeout = false;
		cana_send += sendlong;
		current_a_send = sendlong;
		sendbusy = false;
		sendtime = 0;
	}
	else if(sendtime >= sendtimeout){
		CreateSysTimerInterface()->Stop(S_Monitor);
		for(i = sendbegin;i < sendlong;i++){
			if(a & (1ul << i))
				unsendcount++;
		}
		is_a_s_timeout = true;
		cana_send += sendlong - unsendcount;
		current_a_send = sendlong - unsendcount;
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
			if(mark & (1u << begin)){
				mboxa[begin]->pucMsgData = r_msgdata;
				CANMessageSet(CANA_BASE, begin + 1, mboxa[begin], MSG_OBJ_TYPE_RX);
			}
		}
		CreateSysTimerInterface()->Start(R_Monitor, 1U);
		return eCanSucceed;
	}
}
static void R_Monitor(void){
	recv_time++;
	int i = 0,j = 0;
	if(CANStatusGet(CANA_BASE, CAN_STS_NEWDAT)){
		for(i = recvbegin;i < recv_long;i++){
			if(CANStatusGet(CANB_BASE, CAN_STS_NEWDAT) & (0x1u << i)){
				CANMessageGet(CANA_BASE, i + 1, mboxa[i], true);
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
			is_a_r_timeout = false;
		else if(recv_time >= recv_timeout)
			is_a_r_timeout = true;
		cana_read += recv_count;
		current_a_read = recv_count;
		recvbusy = false;
		recv_count = 0;
		recv_time = 0;
	}
}
///// ---------------------------------interface--------------------------------------
const EcanInterface* EcanaInterface_a(void){
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
