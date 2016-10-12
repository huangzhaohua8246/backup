/*
 * Axiszero6DISK.h
 *
 * Created on: Aug 18, 2016
 * 		Author: hzh
 */

#ifndef ACTION_ZERO_AXISZERO6DISK_H_
#define	ACTION_ZERO_AXISZERO6DISK_H_

#include "zerobase.h"

class Axiszero6DISK		:public zerobase
{
public:
	Axiszero6DISK();
	virtual ~Axiszero6DISK();

//	int Init (void);//原点初始化

	int SingleReturnOrigin (int id);//单轴原点复归
	int SingleStartFindZero (int id);//寻找单轴原点过程
	int SingleReturnStandby (void);//单轴回到设定待机点

	int StartFindZero (void);//开始寻找原点（原点模式下，启动按键调用）
	int ZeroProcess (void);//寻找原点处理过程
	int ReturnStandby (void);//回到设定待机点
	void ReturnZeroStart (void);//原点复归启动（复归模式下，启动按键调用）
//	void ReturnZeroStop (void);//原点复归停止（复归模式下，停止按键调用）

//	int GetZeroStep (void);//读取原点复归步号
//	void StopZero (void);

private:
	bool ReturnZero (void);//回原点
	void ManualSetZero (void);//手动设置原点
	void SetZero (int axis_num);//设置原点
	void ClearRelativePulser (void);
	void ResetPulser (void);
	void ResetAxisPulser (int id);

	int axis_id;
	int axis_ids[6];
	int current_order;
//	int last_p[6];//记录上一次脉冲
	TimerData zero_timer[6];
	int zero_steps[6];//归原点步骤编号
//	INT32 fb_position[6][4];//铁片经过原点时的反馈位置
};

#endif	/* ACTION_ZERO_AXISZERODISK_H_ */
