/******************************************************
** lib_decode.h
** ˵��:
** 1��֧��h264��h265�����ʽ
** 2����ʼ��˳�����Ϊ��1�����캯��
					    2����ʼ������
					    3�����ͱ�������
						4����ȡ��������
** 3�������ʼ���Ժ���Ҫ�޸���ʾ�����ı����ȵ���-ȥ��ʼ������
** 4���쳣�����˳�ʱ��ص�����������
******************************************************/
#ifndef __LIB_DECODE_H_
#define __LIB_DECODE_H_

#include <pthread.h>

//��������ṹ
typedef struct
{
	int width;//����Ŀ�
	int height;//����ĸ�
	int dec_type;//��������ͣ�0-h264��1-h265
	float fps;//����֡�ʣ���ʱֻ�ǲο�
}Dec_Param_T;
//���뷢�����ݽṹ
typedef struct
{
	long long int timestamp;//ʱ���
	int frm_type;//֡���ͣ�sps+pps+I,B,P����1-I֡��0����֡
	int data_len;//�������ݳ���
	void* dataptr;//��������ָ��
}Dec_SendData_T;
//�����ȡ���ݽṹ
typedef struct
{
	long long int timestamp;//ʱ���
	int data_len;//��ȡ���ݳ���
	void* dataptr;//��ȡ����ָ��
}Dec_GetData_T;
//���뻺������ṹ
typedef struct
{
	int sendbuff_size;//���ͻ�������С
	int sendbuff_count;//���ͻ���������
	int getbuff_size;//���ܻ�������С
	int getbuff_count;//���ܻ���������
}DecBuf_Param_T;
//ע�����ͻ�������Ҫ���������û����͵ı������ݣ����ܻ�������Ҫ������Ž��������ݲ����͸��û�

class LibDec
{
public:
	//���캯��
	LibDec();//Ĭ�Ͻ���������Ϊdec0
	LibDec( int ID );//���ݲ�ͬ��IDΪ��������������
	//��������
	~LibDec();
	
	//��ʼ�����뺯��
	int init_dec( DecBuf_Param_T buffParam, Dec_Param_T decParam );//�Զ�������
	//���ͱ�������
	int send_encdata( Dec_SendData_T decSendData );
	//��ȡ��������
	long long int get_decdata( Dec_GetData_T decGetData );
	//ȥ��ʼ�����뺯��
	int reinit_dec();
	
//private://���������޸�Ϊ˽�г�Ա������ͨ���������в����Ķ�д
	char m_caDecName[64];//���������ƣ���Ҫ�������н��������֣��������׳�������
	int m_errcode;//��ʾ��Ĵ����룬0˵����ǰ�޴���
	int m_init;//��ʼ����־
	void* m_ctxptr;//���Ʋ���
	Dec_Param_T m_decParam;//������ز���
	DecBuf_Param_T m_buffParam;//��������ز���
	void* m_sendbuffptr;//���ͻ�����ָ��
	int m_sendbuffridx;//���ͻ�����������
	int m_sendbuffwidx;//���ͻ�����д����
	void* m_getbuffptr;//���ܻ�����ָ��
	int m_getbuffridx;//���ܻ�����������
	int m_getbuffwidx;//���ܻ�����д����
	pthread_t m_dec_loop;//ѭ����ȡ�������ݵ��߳�
};

#endif