/*
 * @Descripttion: Null
 * @version: 1.0
 * @Author: 马平帅
 * @Date: 2020-09-24 08:18:59
 * @LastEditors: Mar Ping
 * @LastEditTime: 2020-11-24 21:09:54
 */



#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    // offsets for PDO entries
    static unsigned int cntlwd;                     //控制字
    static unsigned int ipData;                     //目标位置
    static unsigned int status;                     //状态字
    static unsigned int actpos;                     //当前回授位置
    static unsigned int modes_of_operation;         //6060
    static unsigned int modes_of_operation_display; //6061
    static unsigned int errcode;                    //错误代码
    static unsigned int cur_mode;                   //电机运行模式
    static unsigned int cur_status;                 //电机状态
} ServoMotor;

typedef struct
{
    // offsets for PDO entries
    static unsigned int ECT_Command;                //变频器控制字
    static unsigned int ECT_Frequency;              //变频器频率给定

    static unsigned int ECT_Status;                 //变频器状态字
    static unsigned int ECT_Output_frequency;       //变频器频率反馈
} FreQCDrive;



ServoMotor Motor[10];


int test(int Dirver, unsigned int cnt, unsigned int err){
    Motor[Dirver].cntlwd = cnt;
    Motor[Dirver].errcode = err;
}

test(1,0x80, 0x1996);
