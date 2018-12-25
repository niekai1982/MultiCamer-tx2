#ifndef __CLIENT_H_
#define __CLIENT_H_

#include <stdio.h>

//TCP�ͻ����̣߳��޷���ֵ�������������
//����:pthread_create( &pth, &attr, client_thread, NULL );
void* client_thread( void *arg );

//������־�Զ����·��
//��־�Զ����·����ش������·��
//����˵����
//			pcBuf���Զ����·��ָ��
//			iBufLen���Զ����·������
//����ֵ˵��:
//			0 :�ɹ�
//			-1:��������pcBufΪ�գ�����iBufLen<=0������iBufLen>500
int client_setlogpath( char *pcBuf, int iBufLen );

//����͸����Ϣ
//͸����Ϣ��ش����ַ����������̨�鿴
//����˵����
//			pcBuf��͸����Ϣָ��
//			iBufLen��͸����Ϣ����
//����ֵ˵��:
//			0 :�ɹ�
//			-1:��������pcBufΪ�գ�����iBufLen<=0������iBufLen>5000
int client_createdataevent( char *pcBuf, int iBufLen );

//�����ļ���Ϣ
//�ļ���Ϣ��ش����ļ�����·��
//����˵����
//			pcBuf���ļ�·��ָ��
//			iBufLen���ļ�·������
//����ֵ˵��:
//			0 :�ɹ�
//			-1:��������pcBufΪ�գ�����iBufLen<=0������iBufLen>500
int client_createfileevent( char *pcBuf, int iBufLen );

//�����ļ��ص���������
//����˵����
//			int���ͣ������ļ����ͣ������̶�Ϊ1
//			unsigned char�������ļ�������ɹ��̶�Ϊ0
//����ֵ˵��:�ɻص������Լ�����
typedef int (*CLIENT_DOWNLOAD_CALLBACK)( int, unsigned char );

//���ûص�����
//����˵����
//			fpFunction������CLIENT_DOWNLOAD_CALLBACK���͵ĺ�����
//����ֵ˵��:
//			0 :�ɹ�
//			-1:��������
int client_setdownloadcallback( CLIENT_DOWNLOAD_CALLBACK fpFunction );

//��Ҫ�鿴�����Ǵ򿪣�����Ҫʱ�رոú�
#define __DEBUG__
#ifdef __DEBUG__
#define PRINTF(fmt,args...) printf(fmt, ##args);
#else
#define PRINTF(fmt,args...) 
#endif

#endif