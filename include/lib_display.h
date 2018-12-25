/******************************************************
** lib_display.h
** ˵��:
** 1���ܿ����ƣ�Ŀǰ��ʾֻ֧��nv12��ʽͼ����ʾ������ͨ��
** opencv��ʵ����ɫ���ת����
** 2����ʼ��˳�����Ϊ��1�����캯��
					    2����ʼ������
					    3��������ʾ����
** 3�������ʼ���Ժ���Ҫ�޸���ʾ�����ı����ȵ���-ȥ��ʼ������
** 4���쳣�����˳�ʱ��ص�����������
******************************************************/
#ifndef __LIB_DISPLAY_H_
#define __LIB_DISPLAY_H_

//��ʾ�����ṹ
typedef struct
{
	int width;//��ʾ�Ŀ��
	int height;//��ʾ�ĸ߶�
	int x_offset;//��ʾ��x��ʼ����
	int y_offset;//��ʾ��y��ʼ����
	float fps;//��ʾ��֡�ʣ���ʱֻ�ǲο�
	int frm_type;//֡���ͣ�0=NV12��Ŀǰֻ֧��NV12
	int frm_width;//֡���ݵĿ��
	int frm_height;//֡���ݵĸ߶�
}Display_Param_T;
//��ʾ���ݽṹ
typedef struct
{
	int data_len;//֡���ݳ���
	void *dataptr;//֡����ָ��
}Display_Data_T;
//��������ṹ
typedef struct
{
	int buff_size;//��������С
	int buff_count;//����������
}Buffer_Param_T;

class LibDisplay
{
public:
	//���캯��
	LibDisplay();//Ĭ����ʾ������Ϊrenderer0
	LibDisplay( int ID );//���ݲ�ͬ��IDΪ��ʾ����������
	//��������
	~LibDisplay();
	//��ʼ����ʾ
	int init_display( Buffer_Param_T buffParam );//Ĭ��ȫ����ʾ��Ĭ��֡���ݿ�640��400
	int init_display( Buffer_Param_T buffParam, int displayID );//Ĭ��4������0���ϣ�1���ϣ�2���£�3���£�Ĭ��֡���ݿ�640��400
	int init_display( Buffer_Param_T buffParam, Display_Param_T displayParam );//�Զ�������
	//������ʾ����
	int send_displaydata( Display_Data_T displaydata );
	//ȥ��ʼ��
	int reinit_display();
	
//private://���������޸�Ϊ˽�г�Ա������ͨ���������в����Ķ�д
	char m_caDispName[64];//��ʾ�����ƣ���Ҫ����������ʾ�����֣��������׳�������
	int m_errcode;//��ʾ��Ĵ����룬0˵����ǰ�޴���
	int m_init;//��ʼ����־
	void* m_ctxptr;//���Ʋ���
	Display_Param_T m_displayParam;//��ʾ����ز���
	Buffer_Param_T m_buffParam;//��������ز���
	void* m_buffptr;//������ָ��
	int m_buffridx;//������������
	int m_buffwidx;//������д����
	int m_src_dmafd;//��ʾǰdma�ļ�
};

#endif