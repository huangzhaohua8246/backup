/*
 * Axiszero6DISK.cpp
 *
 *  Created on: Aug 19, 2016
 *      Author: hzh
 */

#include <Axiszero6DISK.h>
#include "data.h"
#include "action.h"
#include "hccommparagenericdef.h"
#include "router.h"
#include "string.h"
#include "zero.h"
#include <stdlib.h>
#include "jog.h"
#include "display.h"
#include "IOTask.h"

Axiszero6DISK::Axiszero6DISK() {
	// TODO Auto-generated constructor stub
	memset(zero_steps,0,6);
}

Axiszero6DISK::~Axiszero6DISK() {
	// TODO Auto-generated destructor stub
}


extern SERVO_LIMIT jog[];

//static void ReadCb(void)
//{
//    my_zero->Set_read_status(true);
//}
//static void ZeroStepCb(void)
//{
//    my_zero->Set_zero_step_cb(true);
//}

//static void ZeroStepDone(void)
//{
//    my_zero->Set_zero_step_delay_done(true);
//}
//static void ZeroDone(void)
//{
//    my_zero->Set_zero_done(true);
//}

//使用 return_single_cb[]函数

//static void ZeroStepCb0(void)
//{
//    my_zero->Set_z_step_cb(0,true);
//}
//static void ZeroStepCb1(void)
//{
//    my_zero->Set_z_step_cb(1,true);
//}
//static void ZeroStepCb2(void)
//{
//    my_zero->Set_z_step_cb(2,true);
//}
//static void ZeroStepCb3(void)
//{
//    my_zero->Set_z_step_cb(3,true);
//}
//static void ZeroStepCb4(void)
//{
//    my_zero->Set_z_step_cb(4,true);
//}
//static void ZeroStepCb5(void)
//{
//    my_zero->Set_z_step_cb(5,true);
//}
//static const Z_END_CB z_cb[6]={
//        ZeroStepCb0,
//        ZeroStepCb1,
//        ZeroStepCb2,
//        ZeroStepCb3,
//        ZeroStepCb4,
//        ZeroStepCb5
//};

//void Axiszero6KEXUYE::Set_z_step_delay_done(int id,bool b){z_step_delay_done[id] = b;}
//static void ZeroStepDone0(void)
//{
//	my_zero->Set_z_step_delay_done(0,true);
//}
//static void ZeroStepDone1(void)
//{
//	my_zero->Set_z_step_delay_done(1,true);
//}
//static void ZeroStepDone2(void)
//{
//	my_zero->Set_z_step_delay_done(2,true);
//}
//static void ZeroStepDone3(void)
//{
//	my_zero->Set_z_step_delay_done(3,true);
//}
//static void ZeroStepDone4(void)
//{
//	my_zero->Set_z_step_delay_done(4,true);
//}
//static void ZeroStepDone5(void)
//{
//	my_zero->Set_z_step_delay_done(5,true);
//}
//static const Z_END_CB z_StepDone[6]={
//		ZeroStepDone0,
//		ZeroStepDone1,
//		ZeroStepDone2,
//		ZeroStepDone3,
//		ZeroStepDone4,
//		ZeroStepDone5
//};

int Axiszero6DISK::SingleReturnOrigin(int id)
{
    return 0;
}

int Axiszero6DISK::SingleStartFindZero(int id)
{
    return 0;
}

int Axiszero6DISK::SingleReturnStandby()
{
    return 0;
}

int Axiszero6DISK::ReturnStandby()
{
    return 0;
}

int Axiszero6DISK::StartFindZero()
{
    int p = GetMachineServoNum();
    zero_start = TRUE;
//    zero_step = 0;
    for (start_id = GX; start_id < p; start_id++)
    {
        pulser_->SetLimit(start_id,0x81,jog[start_id].data_p);//< 设置正极限
        pulser_->SetLimit(start_id,0x82,jog[start_id].data_n);//< 设置负极限
        pulser_->RegisterCalculator(start_id, NULL);
        zero_step_end[start_id] = axis_end[start_id] = z_cnt[start_id] = 0;
    }
    axis_ids[0] = GZ;
    axis_ids[1] = GX;
    axis_ids[2] = GY;
    axis_ids[3] = GU;
    axis_ids[4] = GV;
    axis_ids[5] = GW;
    for(int i = 0;i < GetMachineServoNum();i++)
    	zero_steps[i] = 0;

//    for(int i = 0;i < GetMachineServoNum();i++)
//    	for(int j = 0;i < 4;j++)
//    		fb_position[i][j] = 0;// 反馈位置初始化

    current_order = 0;
    RouterActionStop();
    return 0;
}

void Axiszero6DISK::ReturnZeroStart(void)//< (复归模式下，启动按键调用)原点复归启动
{
    zero_return=true;
    zero_return_done = 0;
    axis_ids[0] = GZ;
    axis_ids[1] = GX;
    axis_ids[2] = GY;
    axis_ids[3] = GU;
    axis_ids[4] = GV;
    axis_ids[5] = GW;
    current_order = zero_step = 0;
    RouterActionStop();
}
//
//void Axiszero6KEXUYE::ReturnZeroStop(void)//< (复归模式下，停止按键调用)原点复归停止
//{
//    zero_return=false;
//    int p = GetMachineServoNum();
//    for (int i = 0; i < p; i++)
//    {
//        pulser_->RegisterCalculator(i, NULL);
//    }
//    RouterActionStop();
//}

void Axiszero6DISK::ClearRelativePulser(void)
{
    switch(all_para->d.P.sys.all[ICAddr_System_Retain_28])
	{
    case 3:
        ClearOrigin();
    case 1:
        memset(relative_pulser, 0, sizeof(relative_pulser));
//        if(!IsOriginSet())
        ResetPulser();
    case 2:all_para->d.P.sys.all[ICAddr_System_Retain_28] = 0;break;
    default:break;
	}
}

void Axiszero6DISK::ResetPulser(void)
{
    for (int i = GX; i < GetMachineServoNum(); i++)
    {
        pulser_->SetOrigin(i,true,0L);
        pulser_->SetOrigin(i,false,0L);
    }
}

void Axiszero6DISK::ResetAxisPulser(int id)
{
    pulser_->SetOrigin(id,true,0L);
    pulser_->SetOrigin(id,false,0L);
}
//#define DIFF_PULSER    50
int Axiszero6DISK::ZeroProcess()
{
    int id = 0;
    if(zero_return)
    {//< 启动原点复归
        if(ReturnZero())
        {
            zero_return = false;
            SetStandbyMode();
        }
        return 0;
    }
    if (zero_done)//原点查找完成后，定时器到时后，zero_done会置为true
    {
        zero_done = false;
        int p = GetMachineServoNum();
        SetZero(p);//所有轴设为原点
        for (id = GX; id < p; id++)
        {
            pulser_->SetLimit(id,jog[id].pos_p,jog[id].data_p);//设置正极限
            pulser_->SetLimit(id,jog[id].pos_n,jog[id].data_n);//设置负极限
            JogInit(id,jog[id]);
        }
    }
    if (!zero_start)
    {
        ManualSetZero();//设原点
        return 0;
    }
    int origin_id;//原点信号
    int speed_ = 20;//速度
    INT32 inch = 0;//增量
    INT32 p_num;//每转脉冲数
    INT16 z_pos=0;//z脉冲数据
    INT16 z_pulser_temp=0;//原点z脉冲
    INT32 zero_target = 0;//目标位置
    INT32 zero_targets[6] = {0};
    static INT32 fb_position[6][4] = {0};//铁片经过原点开关时反馈的位置
    ClearRelativePulser();
    for(int i = GX;i < GetMachineServoNum();)//没有i++
    {
    	if(all_para->d.P.axis_config.para.para[axis_ids[i]].notuse)
    		zero_steps[axis_ids[i]] = 13;//不使用伺服的时候
		switch (zero_steps[axis_ids[i]])
		{
		case 0:
			origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;//原点信号
			p_num = all_para->d.P.axis_config.para.para[axis_ids[i]].ppc;//每转脉冲数
			if(origin_id)
			{
				if(all_para->d.P.axis_config.para.para[axis_ids[i]].origin_dir)//判断方向
					inch = -p_num/4;
				else
					inch = p_num/4;
				if(GetInput(0,origin_id - 1))//判断原点开关，亮，远离原点一段距离，进入50步
				{
					zero_targets[axis_ids[i]] = pulser_->GetPosition(axis_ids[i], NULL)+inch;
					zero_steps[axis_ids[i]] = 50;
				}
				else//判断原点开关，灭，靠近原点一段距离，进入51步
				{
					zero_targets[axis_ids[i]] = pulser_->GetPosition(axis_ids[i], NULL)-inch;
					zero_steps[axis_ids[i]] = 51;
				}
		    	speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
				RouterSetSingleSpeed(axis_ids[i], speed_);//设定运动速度
				axis_end[axis_ids[i]] = false;
				RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
				RouterSingleMoveTo(axis_ids[i], zero_targets[axis_ids[i]]);//开始发脉冲
			}
			else
				zero_steps[axis_ids[i]] = 13;//该轴没有设定原点直接跳过
			break;
		case 50:
			if (axis_end[axis_ids[i]])//等待运动完成
			{
				origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
				if (GetInput(0,origin_id - 1))
				{//判断原点开关，亮，进入0步（亮到亮，重复）
					zero_steps[axis_ids[i]] = 0;
				}
				else
				{//灭，进入1步（亮到灭，完成）
					zero_steps[axis_ids[i]] = 1;
				}
			}
			break;
		case 51:
			if (axis_end[axis_ids[i]])//等待运动完成
			{
				origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
				if (GetInput(0,origin_id - 1))
				{//判断原点开关，亮，进入0步（灭到亮，完成）
					zero_steps[axis_ids[i]] = 0;
				}
				else
				{//灭，进入0步（灭到灭，重复）
					zero_steps[axis_ids[i]] = 0;
				}
			}
			break;
		case 1://让轴远离原点开关一段距离，进入2步
			p_num = all_para->d.P.axis_config.para.para[axis_ids[i]].ppc;
			speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
			RouterSetSingleSpeed(axis_ids[i], speed_);
			axis_end[axis_ids[i]] = false;
			RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
			RouterSingleMoveTo(axis_ids[i], pulser_->GetPosition (axis_ids[i], NULL) + p_num/4);
			zero_steps[axis_ids[i]]	= 2;
			break;
		case 2://等待运动完成
			if (axis_end[axis_ids[i]])
			{
				origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
				if (!GetInput(0,origin_id - 1))
				{//判断原点开关，灭，进入3步（灭到灭，完成）
					zero_steps[axis_ids[i]] = 3;
				}
				else
				{//灭，进入0步（灭到亮，之前步骤失败）
					zero_steps[axis_ids[i]] = 0;
				}
			}
			break;
		case 3://让轴向原点开关移动一大段距离，进入4步
			p_num = all_para->d.P.axis_config.para.para[axis_ids[i]].ppc;
			speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
			RouterSetSingleSpeed(axis_ids[i], speed_);
			axis_end[axis_ids[i]] = false;
			RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
			RouterSingleMoveTo(axis_ids[i], pulser_->GetPosition (axis_ids[i], NULL) - p_num*100);
			zero_steps[axis_ids[i]]	= 4;
			break;
		case 4:
			origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
			if (GetInput(0,origin_id - 1))//判断原点开关
			{//原点开关 灭到亮时，记录当前反馈位置0
				pulser_->GetPosition (axis_ids[i], &fb_position[axis_ids[i]][0]);
				zero_steps[axis_ids[i]] = 21;//进入21
			}
			break;
		case 21:
			origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
			if (!GetInput(0,origin_id - 1))//判断原点开关
			{//原点开关 亮到灭，记录当前反馈位置1
				pulser_->GetPosition (axis_ids[i], &fb_position[axis_ids[i]][1]);
				//停止当前轴运动，进入5步
				axis_end[axis_ids[i]] = false;
				RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
				RouterSingleStop(axis_ids[i], true);
				zero_steps[axis_ids[i]] = 5;
			}
			break;
		case 5://等待运动完成，进入6步
			if (axis_end[axis_ids[i]])
				zero_steps[axis_ids[i]] = 6;
			break;
		case 6://向原点开关移动一段距离，进入7步
			p_num = all_para->d.P.axis_config.para.para[axis_ids[i]].ppc;
			speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
			RouterSetSingleSpeed(axis_ids[i], speed_);
			axis_end[axis_ids[i]] = false;
			RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
			RouterSingleMoveTo(axis_ids[i], pulser_->GetPosition (axis_ids[i], NULL) + p_num/4);
			zero_steps[axis_ids[i]] = 7;
			break;
		case 7://等待运动完成
			if (axis_end[axis_ids[i]])
			{
				origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
				if (!GetInput(0, origin_id - 1))
				{//判断原点开关，灭，进入8步（灭到灭，完成）
					zero_steps[axis_ids[i]] = 8;
				}
				else
				{//亮，进入0步（灭到亮，之前步骤失败）
					zero_steps[axis_ids[i]] = 0;
				}
			}
			break;
		case 8://让轴远离原点开关移动一大段距离，进入9步
			p_num = all_para->d.P.axis_config.para.para[axis_ids[i]].ppc;
			speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
			RouterSetSingleSpeed(axis_ids[i], speed_);
			axis_end[axis_ids[i]] = false;
			RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
			RouterSingleMoveTo(axis_ids[i], pulser_->GetPosition (axis_ids[i], NULL) + p_num*100);
			zero_steps[axis_ids[i]]	= 9;
			break;
		case 9:
			origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
			if (GetInput(0,origin_id - 1))//判断原点开关
			{//原点开关 灭到亮时，记录当前反馈位置2
				pulser_->GetPosition (axis_ids[i], &fb_position[axis_ids[i]][2]);
				zero_steps[axis_ids[i]] = 23;//进入23
			}
			break;
		case 23:
			origin_id = all_para->d.P.axis_config.para.para[axis_ids[i]].origin;
			if (!GetInput(0,origin_id - 1))//判断原点开关
			{//原点开关 亮到灭时，记录当前反馈位置3
				pulser_->GetPosition (axis_ids[i], &fb_position[axis_ids[i]][3]);
				//停止当前轴运动，进入10步
				axis_end[axis_ids[i]] = false;
				RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
				RouterSingleStop(axis_ids[i],true);
				zero_steps[axis_ids[i]] = 10;
			}
			break;
		case 10://等待运动完成，进入11步
			if (axis_end[axis_ids[i]])
				zero_steps[axis_ids[i]] = 11;
			break;
		case 11://归原点
			speed_ = (INT32)all_para->d.P.axis_config.para.para[axis_ids[i]].origin_speed*all_para->d.P.axis_config.para.para[axis_ids[i]].max_speed*0.001;
			RouterSetSingleSpeed(axis_ids[i], speed_);
			axis_end[axis_ids[i]] = false;
			RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
			RouterSingleMoveTo(axis_ids[i], ((fb_position[axis_ids[i]][0] + fb_position[axis_ids[i]][1] + fb_position[axis_ids[i]][2] + fb_position[axis_ids[i]][3])/4));//回到原点位置
			zero_steps[axis_ids[i]]	= 12;
			break;
		case 12:
			if (axis_end[axis_ids[i]])//等待运动完成
				zero_steps[axis_ids[i]] = 13;//进入13
			break;
		case 13:
			zero_step_end[axis_ids[i]] = 1;//开始找z脉冲
			zero_steps[axis_ids[i]] = 14;//当前轴找原点完成
			break;
		default:
			break;
		}//switch
		if(zero_steps[axis_ids[0]] == 14)//先将axis_ids[0]代表的轴的动作完成后，在去走其他的轴，其他的轴同时动作
			i++;
		else
			break;
    }//for

    int result =0;
    for(id=0;id<GetMachineServoNum();id++)
    {
    	if(all_para->d.P.axis_config.para.para[id].notuse)
    		zero_step_end[id] = 100;//不使用伺服的时候
		p_num = all_para->d.P.axis_config.para.para[id].ppc;//每转脉冲数
		switch(zero_step_end[id])
		{
		case 0://此步说明轴还没有运动到原点附近位置，等待轴走到原点附近
			break;
		case 1:
		{
			origin_id = all_para->d.P.axis_config.para.para[id].origin;
			if(!origin_id)
				zero_step_end[id] = 100;
			INT32 out_p, in_p;
			out_p = pulser_->GetPosition(id, &in_p);//当前脉冲数，反馈回来的脉冲数
			if (labs(out_p - in_p)<=500)
			{//在误差范围内
				z_cnt[id]++;
				if (z_cnt[id] > p_num)
				{
					z_cnt[id] = 0;
					zero_step_end[id] = 2;
				}
			}
		}
			break;
		case 2:
			z_pulser_temp = (INT16)z_pulser[id];//得到原点位置的z脉冲偏差
			z_pos = pulser_->Single(id);//当前位置的z脉冲偏差
			origin_id = all_para->d.P.axis_config.para.para[id].origin;
			p_num = all_para->d.P.axis_config.para.para[id].ppc;//每转脉冲数
			if ((z_pulser_temp == z_pos)
					|| (((INT16)(z_pulser_temp - p_num)) == z_pos)
					|| (((INT16)(p_num + z_pulser_temp)) == z_pos))//当前位置和原点位置重合的时候
			{
				zero_step_end[id] = 100;//查找原点位置完成
			}
			else
			{//不重合，计算两个位置的z偏差差值
				if (z_pos >= 0)
				{
					if (z_pulser_temp >= 0)
					{
						zero_target = z_pulser_temp - z_pos;
					}
					else
					{
						zero_target = p_num + z_pulser_temp - z_pos;
					}
				}
				else
				{
					if (z_pulser_temp >= 0)
					{
						zero_target = z_pulser_temp - p_num - z_pos;
					}
					else
					{
						zero_target = z_pulser_temp - z_pos;
					}
				}
				if(labs(zero_target) > p_num/10)//判断误差
				{
					errPLC = ALARM_AXIS1_ORIGIN_DEVIATION + id;//误差较大，报错
					zero_step_end[id] = 98;//只有在这里会进入98
				}
				else
				{//在误差范围内
					axis_end[id] = false;
					if(!RouterSetSingleSpeed(id, 20))errPLC = ALARM_AXIS_SPEED_SET_ERR + id;
					RouterRegisterSingleMotionEndCb(id, return_single_cb[id]);
					SetOriginStartAndEndSpeed(id);
					if(!RouterSingleMoveTo(id,zero_target + pulser_->GetPosition(id, NULL)))//减去差值，轴可以转到上个原点的位置
						errPLC = ALARM_AXIS_RUN_ERR + id;
					zero_step_end[id] = 100;//运动完成
				}
			}
			break;
		case 98:
			if(all_para->all[ICAddr_System_Retain_28] == 4)
			{//用户按下 重新设原点的选项
				errPLC = ALARM_NULL;//清除报警
				for(int j = 0;j < GetMachineServoNum();j++)
				{
					if(zero_step_end[j] < 98)//后面的轴重新查找原点
					{
						zero_steps[j] = 0;
						zero_step_end[j] = 0;
					}
				}
				DataWriteAddr_32(ICAddr_System_Retain_28,0);//写入0,取消回原点
				zero_step_end[id]++;//进入99
			}
			break;
		case 99://原点查找完成后，将查找的最后一个原点位置偏差的轴重新设为原点，如果有三个轴的原点位置偏移，就重复查找三次原点，每次查找原点会将最后一个轴的原点位置重新设为原点
			DataWriteAddr_32(ICAddr_System_Retain_1,CMD_SET_ZERO0 + id);//查找三次就可以将三个轴重新设为原点，第四次查找时将不会出现原点位置偏差报警
			zero_step_end[id]++;//进入100
			break;
		case 100:
			result++;
			break;
		default:
			break;
		}//switch()
    }//for()
    if(result == GetMachineServoNum())
    {//全部原点查找完成
        zero_done = zero_start = false;//将这两个关键标志位设为false
        TimerStart(&zero_timer[0], 1, 2000, ZeroDone);//定时器到时后，会将zero_done置为true
    }
    return 0;
}

//void Axiszero6DISK::StopZero()//< 停止归原点（原点模式下，停止按键调用）
//{
//    int p = GetMachineServoNum();
//    zero_start = FALSE;
//    for (start_id = GX; start_id < p; start_id++)
//    {
////        pulser_->SetLimit(start_id,jog[start_id].pos_p,jog[start_id].data_p);//< 设置正极限
////        pulser_->SetLimit(start_id,jog[start_id].pos_n,jog[start_id].data_n);//< 设置负极限
//        pulser_->RegisterCalculator(start_id, NULL);
//    }
//}

bool Axiszero6DISK::ReturnZero(void)
{//复归启动
    if(!RouterRouteIsDone())return true;//运动完成，执行下面的
    int port = GetMachineServoNum();
    int ret = 0;
    FLOAT32 temp = (FLOAT32)all_para->d.P.interpolation.para.p[0].speed_percent*0.001;
    int speed_ = 50;
    switch(zero_step)
    {
    case 0://先将axis_ids[0]代表的轴回到原点位置
    	speed_ = temp*all_para->d.P.axis_config.para.para[0].max_speed*0.1;
    	if(speed_>300)
    		speed_=300;
        RouterSetSingleSpeed(axis_ids[0], speed_);
        zero_step_cb = false;
        RouterRegisterSingleMotionEndCb(axis_ids[0], ZeroStepCb);
		SetOriginStartAndEndSpeed(axis_ids[0]);
        RouterSingleMoveTo(axis_ids[0], 0);
        zero_step++;
        break;
    case 1://再先将剩下的axis_ids[i]代表的轴回到原点位置
        if(zero_step_cb)
        {
            current_order++;
            for(int i=1;i<GetMachineServoNum();i++)
            {
            	speed_ = temp*all_para->d.P.axis_config.para.para[i].max_speed*0.1;
            	if(speed_>300)speed_=300;
                RouterSetSingleSpeed(axis_ids[i], speed_);
            	Set_axis_end(axis_ids[i],false);
                RouterRegisterSingleMotionEndCb(axis_ids[i], return_single_cb[axis_ids[i]]);
                SetOriginStartAndEndSpeed(axis_ids[i]);
                RouterSingleMoveTo(axis_ids[i], 0);
            }
            zero_step++;
        }
        break;
    case 2://复归完成
    	ret = 1;
        for(int i=1;i<GetMachineServoNum();i++)
        {
        	bool e = Get_axis_end(axis_ids[i]);
        	if(e == false)
        	{
        		ret = 0;
        		break;
        	}
        }
        break;
    default:
    	break;
    }
    return ret;
}
void Axiszero6DISK::ManualSetZero(void)
{
    INT16 set, id, i;
    INT32 val;
    set = DataReadAddr_32(ICAddr_System_Retain_1);
    switch (set)
    {
    case CMD_SET_ZERO0://X轴原点设定
    case CMD_SET_ZERO1://Y轴原点设定
    case CMD_SET_ZERO2://Z轴原点设定
    case CMD_SET_ZERO3://U轴原点设定
    case CMD_SET_ZERO4://V轴原点设定
    case CMD_SET_ZERO5://W轴原点设定
        val = GetMachineServoNum();
        id = set - CMD_SET_ZERO0;//轴的id
        if (id > val) break;
        z_pulser[id] = pulser_->Single(id);//当前轴位置的z脉冲偏差
        read_status = false;
        while (persist_->Write(ZPULSER_STORAGER_START_ADDR + id, 1,
                z_pulser + id, ReadCb) != 1);//写入数据
        pulser_->SetOrigin(id, TRUE, 0L);//设原点
        relative_pulser[id] = pulser_->GetPosition(id, NULL);//当前位置
        do
        {
        } while (!read_status);
        read_status = false;
        while (persist_->Write(POS_STORAGER_START_ADDR + 2*id, 2,
                (UINT16*) (relative_pulser + id), ReadCb) != 2);//写入数据
        do
        {
        } while (!read_status);

        read_status = false;
        relative_pulser[9] |= 1<<id;
        while (persist_->Write(POS_STORAGER_STATUS_ADDR, 2,
                (UINT16*) (relative_pulser + 9), ReadCb) != 2);
        do
        {
        } while (!read_status);


        DataWriteAddr_32(ICAddr_System_Retain_1, 0);//将0写入这个地址，意味着此轴已设定完毕
        break;
    case CMD_SET_ZERO: //< 全部轴原点设定
        val = GetMachineServoNum();
        for (id = 0; id < val; id++)
        {
            z_pulser[id] = pulser_->Single(id);//当前位置的z脉冲偏差
            pulser_->SetOrigin(id, TRUE, 0L);//设原点
            relative_pulser[id] = pulser_->GetPosition(id, NULL);//当前位置
        }
        i = val;
        read_status = false;
        while (persist_->Write(ZPULSER_STORAGER_START_ADDR, i,
                z_pulser, ReadCb) != i);//写入z脉冲偏差
        do
        {
        } while (!read_status);
        i = POS_STORAGER_LENTH;
        relative_pulser[9] = 0xff;//记录位置
        read_status = false;
        while (persist_->Write(POS_STORAGER_START_ADDR, i,
                (UINT16*) relative_pulser, ReadCb) != i);//写入位置数据
        do
        {
        } while (!read_status);
        zero_done = true;
        DataWriteAddr_32(ICAddr_System_Retain_1, 0);//写入0,表示设定完毕
        break;
    case CMD_REM_POS: //保存每个轴当前位置
        if (IsOriginSet())//原点设定完成
        {
            val = GetMachineServoNum();
            for (id = 0; id < val; id++)
            {
                pulser_->GetPosition(id, relative_pulser+id);//当前位置
            }
            i = POS_STORAGER_LENTH;
            relative_pulser[9] = 0xFF;//记录位置
            while (persist_->Write(POS_STORAGER_START_ADDR, i,
                    (UINT16*) relative_pulser, ReadCb) != i);//写入当前位置数据
        }
        DataWriteAddr_32(ICAddr_System_Retain_1, 0);//写入0,表示保存完毕
        break;
    default:
        return;
    }
    RouterPulserChanged();
}

void Axiszero6DISK::SetZero(int axis_num)
{
    RouterPulserChanged();
    for (int id = 0; id < axis_num; id++)
    {
        relative_pulser[id] = 0;//当前位置设为0
        pulser_->SetOrigin(id, TRUE, 0L);//设为原点
    }
    relative_pulser[9] = 0xff;//记录位置
    int i = sizeof(relative_pulser);
    read_status = FALSE;
    while (persist_->Write(POS_STORAGER_START_ADDR, i,
            (UINT16*) relative_pulser, ReadCb) != i)
        ;//写入位置数据
    do
    {
    } while (!read_status);
    SetStandbyMode();
}
