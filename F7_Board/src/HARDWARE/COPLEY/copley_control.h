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
void creat_sdo_msg(can_msg_t msg,   //��Ϣָ��
	                 uint32_t node_id,//node id
                   uint32_t op, //����д
                   uint16_t index, //����
                   uint8_t subindex,//������
                   uint8_t* buf,uint8_t len);//���ݺͳ���





typedef struct{
	
		const uint32_t node_id; //�ڵ��
		const uint32_t rpdo_id[5];//RPOD��ʶ
		const uint32_t tpdo_id[5];//TPOD��ʶ
		const uint32_t heart_beat;
		const uint32_t sdo_id;
	
		//device_t copley_dev;       //�豸����ָ��
	
		uint16_t control_word;  //������
		uint16_t status_word ;   //״̬��
		uint32_t status_register; //����Ĵ���
		uint8_t nmt_state ;    //NMT״̬
		uint8_t operation_mode; //����ģʽ
		uint16_t error_code;  //������
		uint32_t actual_target_p;
		int32_t csp_target;  //λ��Ŀ��
		int32_t csv_target;  //�ٶ�Ŀ��
		int32_t cst_target;  //����Ŀ��
		int32_t actual_p;   //ʵ��λ��
		int32_t actual_v;   //ʵ���ٶ�
		int32_t actual_t;   //ʵ������
		void(*update)(can_msg_t rcv_msg); //������Ϣ���ո��º���              
	  void(*start)(uint8_t mode);
	  void(*falut_reset)(void);       //����λ����
	  int(*set_target)( uint8_t terget, int32_t val);//����Ŀ�꺯��
	  void(*nmt_reset)(void);          //NMT��λ����
		void(*state_reset)(void);      //״̬��λ����
		void(*quick_stop)(void);       //��ͣ����
		void(*init)();                 //��ʼ������
		void(*set_op_mode)(uint8_t mode);
		void(*sync)(void);   //����ͬ�������  
		void(*ppm_init)(void); //PPMģʽ��ʼ������
		void(*ppm_set_target)(int target); //PPMģʽ����Ŀ�꺯��
		void(*halt)(void); //ֹͣ�˶�
} copley_device;


//����Ŀ��
#define TARGET_CSP 1
#define TARGET_CSV 2
#define TARGET_CST 3

//��д�ֵ����
#define SDO_READ_OD 1
#define SDO_WRITE_OD 2

//NMT״̬������
#define NMT_START_NODE  0x01
#define NMT_STOP_NODE   0x02
#define NMT_ENTER_OP    0x80
#define RESET_NODE      0x81
#define ERSET_COMMU     0x82


//NMT״̬
#define NMT_STA_INI    0
#define NMT_STA_STOP   4
#define NMT_STA_OPER   5
#define NMT_STA_PRE_OP 127

//�������ģʽ
#define OP_MODE_PPM 1
#define OP_MODE_PVM 3
#define OP_MODE_HMM 6
#define OP_MODE_CSP 8
#define OP_MODE_CSV 9
#define OP_MODE_CST 10




//����֡��COD-ID
#define DEVICE1_TPDO4  0x4A1 
#define DEVICE1_TPDO3 0x3A1
#define DEVOCE1_HERTBEAT 0x701
#define DEVICE1_SDO_RESP 0x581




//������
#define EN_VOL_BIT 0x02    //����ѹ�����ȣ���ѹһ�أ���������ȫʧ��
#define QK_STOP_BIT 0x04   //QK STOP���ڿ���ѹ������µĹ��ܣ�QK STOP�򿪣���������Ҳʧ��,0��ƽ��Ч
#define SWITCH_ON_BIT 0x01 //����
#define EN_OP_BIT 0x08     //�ڿ��ش򿪺�����ܷ����
#define FAULT_BIT 0x80     //�������󣬽��и�λ


#define CMD_EN_VOL  EN_VOL_BIT|QK_STOP_BIT
#define CMD_SW_ON   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT
#define CMD_EN_OP   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT|EN_OP_BIT

#define CMD_QK_STOP EN_VOL_BIT  //�ڵ�ѹ���ο��£�QK Stopλ��0�����Ǽ�ͣ����
#define CMD_DIS_VOL 0x00

#define FAULT_RST FAULT_BIT


#define CTL_W_HALT_BIT 0x0100

#define CTL_W_MODE_BITS_MASK 0x0070
#define CTL_W_PPM_INI_BIT 0x0010
#define CTL_W_PPM_CHA_IMM_BIT 0x0020
#define CTL_W_PPM_ABS_REL_BIT 0x0040




//״̬�궨��


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
