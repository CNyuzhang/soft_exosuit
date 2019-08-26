#include "usmart.h"
#include "usmart_str.h"
#include "copley_node1.h"
#include "copley_node2.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����) 
#include "delay.h"	 
#include "can.h"  


#include "test.h"

//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",	 
#endif		     
	(void*)node1_ppm_set_target,"void node1_ppm_set_target(int target)",
	(void*)node2_ppm_set_target,"void node2_ppm_set_target(int target)",
	(void*)node2_halt	,"void node2_halt(void)",
	(void*)node1_halt	,"void node1_halt(void)",
	(void*)set_start,"void set_start(int flag)",
	(void*)set_start2,"void set_start2(int flag)",
	(void*)set_kd,"void set_kd(int kd)",
	(void*)set_kp,"void set_kp(int kp)",
	(void*)show_pid,"void show_pid(void)",
	(void*)read_sdo,"void read_sdo(uint32_t index,uint32_t subindex)",
	(void*)set_sdo,"void set_sdo(void)",
	(void*)send_sync,"void send_sync(void)",
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
	0,	  	//��������
	0,	 	//����ID
	1,		//������ʾ����,0,10����;1,16����
	0,		//��������.bitx:,0,����;1,�ַ���	    
	0,	  	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
	0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};   



















