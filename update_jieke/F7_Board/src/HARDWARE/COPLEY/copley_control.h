#ifndef __COPLEY_CONTROL_H__
#define __COPLEY_CONTROL_H__
#include "sys.h"
#include "can.h"
void pdo_map(uint8_t node_id,uint16_t index,uint8_t sub_index,uint32_t map_data);
void pdo_enable(uint8_t node_id,uint16_t id_index,uint16_t cob_id,uint16_t n_index,uint8_t num);
void pdo_disable(uint8_t node_id,uint16_t id_index,uint16_t n_index);
void pdo_set_type(uint8_t node_id,uint16_t index,uint8_t type);

void creat_nmt_msg(can_msg_t msg,uint32_t node_id,uint8_t op);
void creat_sync_msg(can_msg_t msg);
void send_sync(void);
void create_rpdo_msg(can_msg_t msg,
	                   uint32_t cob_id,
										 uint8_t* buf,
                     uint8_t len);
void creat_sdo_msg(can_msg_t msg,   //信息指针
	                 uint32_t node_id,//node id
                   uint32_t op, //读或写
                   uint16_t index, //索引
                   uint8_t subindex,//子索引
                   uint8_t* buf,uint8_t len);//数据和长度





typedef struct{
	
		const uint32_t node_id; //节点号
		const uint32_t rpdo_id[5];//RPOD标识
		const uint32_t tpdo_id[5];//TPOD标识
		const uint32_t heart_beat;
		const uint32_t sdo_id;
	
		//device_t copley_dev;       //设备对象指针
	
		uint16_t control_word;  //控制字
		uint16_t status_word ;   //状态字
		uint32_t status_register; //错误寄存器
		uint8_t nmt_state ;    //NMT状态
		uint8_t operation_mode; //工作模式
		uint16_t error_code;  //错误码
		uint32_t actual_target_p;
		int32_t csp_target;  //位置目标
		int32_t csv_target;  //速度目标
		int32_t cst_target;  //力矩目标
		int32_t actual_p;   //实际位置
		int32_t actual_v;   //实际速度
		int32_t actual_t;   //实际力矩
		void(*update)(can_msg_t rcv_msg); //反馈信息接收更新函数              
	  void(*start)(uint8_t mode);
	  void(*falut_reset)(void);       //错误复位函数
	  int(*set_target)( uint8_t terget, int32_t val);//设置目标函数
	  void(*nmt_reset)(void);          //NMT复位函数
		void(*state_reset)(void);      //状态复位函数
		void(*quick_stop)(void);       //快停函数
		void(*init)();                 //初始化函数
		void(*set_op_mode)(uint8_t mode);
		void(*sync)(void);   //发送同步命令函数  
		void(*ppm_init)(void); //PPM模式初始化函数
		void(*ppm_set_target)(int target); //PPM模式设置目标函数
		void(*halt)(void); //停止运动
} copley_device;


//操作目标
#define TARGET_CSP 1
#define TARGET_CSV 2
#define TARGET_CST 3

//读写字典操作
#define SDO_READ_OD 1
#define SDO_WRITE_OD 2

//NMT状态机命令
#define NMT_START_NODE  0x01
#define NMT_STOP_NODE   0x02
#define NMT_ENTER_OP    0x80
#define RESET_NODE      0x81
#define ERSET_COMMU     0x82


//NMT状态
#define NMT_STA_INI    0
#define NMT_STA_STOP   4
#define NMT_STA_OPER   5
#define NMT_STA_PRE_OP 127

//电机操作模式
#define OP_MODE_PPM 1
#define OP_MODE_PVM 3
#define OP_MODE_HMM 6
#define OP_MODE_CSP 8
#define OP_MODE_CSV 9
#define OP_MODE_CST 10




//数据帧的COD-ID
#define DEVICE1_TPDO4  0x4A1 
#define DEVICE1_TPDO3 0x3A1
#define DEVOCE1_HERTBEAT 0x701
#define DEVICE1_SDO_RESP 0x581




//命令定义宏
#define EN_VOL_BIT 0x02    //开电压最优先，电压一关，其他功能全失灵
#define QK_STOP_BIT 0x04   //QK STOP是在开电压的情况下的功能，QK STOP打开，其他功能也失灵,0电平有效
#define SWITCH_ON_BIT 0x01 //开关
#define EN_OP_BIT 0x08     //在开关打开后决定能否操作
#define FAULT_BIT 0x80     //发生错误，进行复位


#define CMD_EN_VOL  EN_VOL_BIT|QK_STOP_BIT
#define CMD_SW_ON   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT
#define CMD_EN_OP   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT|EN_OP_BIT

#define CMD_QK_STOP EN_VOL_BIT  //在电压打开形况下，QK Stop位置0，就是急停命令
#define CMD_DIS_VOL 0x00

#define FAULT_RST FAULT_BIT


#define CTL_W_HALT_BIT 0x0100

#define CTL_W_MODE_BITS_MASK 0x0070
#define CTL_W_PPM_INI_BIT 0x0010
#define CTL_W_PPM_CHA_IMM_BIT 0x0020
#define CTL_W_PPM_ABS_REL_BIT 0x0040




//状态宏定义


#define DEV_STA_SET_P_ACK  0x1000
#define DEV_STA_TAR_REA 0x0400

#define DEV_STATE_MSDK  0x006F

#define DEV_STA_NOT_RD_TO_SW_ON 0x0000
#define DEV_STA_SW_ON_DISABLE 0x0040
#define DEV_STA_RD_TO_SW_ON 0x0021
#define DEV_STA_SWITCH_ON 0x0023
#define DEV_STA_OPER_EN 0x0027
#define DEV_STA_QK_STOP 0x0007
#define DEV_STA_FLT_RE_AC 0x000F
#define DEV_STA_FAULT 0x0008


#endif
