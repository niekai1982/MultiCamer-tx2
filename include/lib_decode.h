/******************************************************
** lib_decode.h
** 说明:
** 1）支持h264和h265编码格式
** 2）初始化顺序必须为：1、构造函数
					    2、初始化函数
					    3、发送编码数据
						4、获取解码数据
** 3）如果初始化以后想要修改显示参数的必须先调用-去初始化函数
** 4）异常处理退出时务必调用析构函数
******************************************************/
#ifndef __LIB_DECODE_H_
#define __LIB_DECODE_H_

#include <pthread.h>

//解码参数结构
typedef struct
{
	int width;//解码的宽
	int height;//解码的高
	int dec_type;//解码的类型，0-h264，1-h265
	float fps;//编码帧率，暂时只是参考
}Dec_Param_T;
//解码发送数据结构
typedef struct
{
	long long int timestamp;//时间戳
	int frm_type;//帧类型（sps+pps+I,B,P），1-I帧，0其他帧
	int data_len;//发送数据长度
	void* dataptr;//发送数据指针
}Dec_SendData_T;
//解码获取数据结构
typedef struct
{
	long long int timestamp;//时间戳
	int data_len;//获取数据长度
	void* dataptr;//获取数据指针
}Dec_GetData_T;
//解码缓冲参数结构
typedef struct
{
	int sendbuff_size;//发送缓冲区大小
	int sendbuff_count;//发送缓冲区个数
	int getbuff_size;//接受缓冲区大小
	int getbuff_count;//接受缓冲区个数
}DecBuf_Param_T;
//注：发送缓冲区主要用来接受用户发送的编码数据，接受缓冲区主要用来存放解码后的数据并发送给用户

class LibDec
{
public:
	//构造函数
	LibDec();//默认解码器名称为dec0
	LibDec( int ID );//根据不同的ID为解码器赋予名字
	//析构函数
	~LibDec();
	
	//初始化解码函数
	int init_dec( DecBuf_Param_T buffParam, Dec_Param_T decParam );//自定义设置
	//发送编码数据
	int send_encdata( Dec_SendData_T decSendData );
	//获取解码数据
	long long int get_decdata( Dec_GetData_T decGetData );
	//去初始化解码函数
	int reinit_dec();
	
//private://后续可以修改为私有成员变量，通过函数进行参数的读写
	char m_caDecName[64];//解码器名称，主要用来进行解码器区分，否则容易出现问题
	int m_errcode;//显示类的错误码，0说明当前无错误
	int m_init;//初始化标志
	void* m_ctxptr;//控制参数
	Dec_Param_T m_decParam;//解码相关参数
	DecBuf_Param_T m_buffParam;//缓冲区相关参数
	void* m_sendbuffptr;//发送缓冲区指针
	int m_sendbuffridx;//发送缓冲区读索引
	int m_sendbuffwidx;//发送缓冲区写索引
	void* m_getbuffptr;//接受缓冲区指针
	int m_getbuffridx;//接受缓冲区读索引
	int m_getbuffwidx;//接受缓冲区写索引
	pthread_t m_dec_loop;//循环读取编码数据的线程
};

#endif