#ifndef __CLIENT_H_
#define __CLIENT_H_

#include <stdio.h>

//TCP客户端线程，无返回值，无需输入变量
//例如:pthread_create( &pth, &attr, client_thread, NULL );
void* client_thread( void *arg );

//设置日志自动检测路径
//日志自动检测路径务必传输绝对路径
//参数说明：
//			pcBuf：自动检测路径指针
//			iBufLen：自动检测路径长度
//返回值说明:
//			0 :成功
//			-1:参数错误，pcBuf为空，或者iBufLen<=0，或者iBufLen>500
int client_setlogpath( char *pcBuf, int iBufLen );

//产生透传消息
//透传消息务必传输字符串，方便后台查看
//参数说明：
//			pcBuf：透传消息指针
//			iBufLen：透传消息长度
//返回值说明:
//			0 :成功
//			-1:参数错误，pcBuf为空，或者iBufLen<=0，或者iBufLen>5000
int client_createdataevent( char *pcBuf, int iBufLen );

//产生文件消息
//文件消息务必传输文件绝对路径
//参数说明：
//			pcBuf：文件路径指针
//			iBufLen：文件路径长度
//返回值说明:
//			0 :成功
//			-1:参数错误，pcBuf为空，或者iBufLen<=0，或者iBufLen>500
int client_createfileevent( char *pcBuf, int iBufLen );

//下载文件回调函数类型
//参数说明：
//			int类型：下载文件类型，升级固定为1
//			unsigned char：下载文件结果，成功固定为0
//返回值说明:由回调函数自己管理
typedef int (*CLIENT_DOWNLOAD_CALLBACK)( int, unsigned char );

//设置回调函数
//参数说明：
//			fpFunction：符合CLIENT_DOWNLOAD_CALLBACK类型的函数名
//返回值说明:
//			0 :成功
//			-1:参数错误
int client_setdownloadcallback( CLIENT_DOWNLOAD_CALLBACK fpFunction );

//需要查看调试是打开，不需要时关闭该宏
#define __DEBUG__
#ifdef __DEBUG__
#define PRINTF(fmt,args...) printf(fmt, ##args);
#else
#define PRINTF(fmt,args...) 
#endif

#endif