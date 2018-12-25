/******************************************************
** lib_display.h
** 说明:
** 1）受库限制，目前显示只支持nv12格式图像显示，可以通过
** opencv库实现颜色域的转换。
** 2）初始化顺序必须为：1、构造函数
					    2、初始化函数
					    3、发送显示数据
** 3）如果初始化以后想要修改显示参数的必须先调用-去初始化函数
** 4）异常处理退出时务必调用析构函数
******************************************************/
#ifndef __LIB_DISPLAY_H_
#define __LIB_DISPLAY_H_

//显示参数结构
typedef struct
{
	int width;//显示的宽度
	int height;//显示的高度
	int x_offset;//显示的x起始坐标
	int y_offset;//显示的y起始坐标
	float fps;//显示的帧率，暂时只是参考
	int frm_type;//帧类型，0=NV12，目前只支持NV12
	int frm_width;//帧数据的宽度
	int frm_height;//帧数据的高度
}Display_Param_T;
//显示数据结构
typedef struct
{
	int data_len;//帧数据长度
	void *dataptr;//帧数据指针
}Display_Data_T;
//缓冲参数结构
typedef struct
{
	int buff_size;//缓冲区大小
	int buff_count;//缓冲区个数
}Buffer_Param_T;

class LibDisplay
{
public:
	//构造函数
	LibDisplay();//默认显示器名称为renderer0
	LibDisplay( int ID );//根据不同的ID为显示器赋予名字
	//析构函数
	~LibDisplay();
	//初始化显示
	int init_display( Buffer_Param_T buffParam );//默认全屏显示，默认帧数据宽640高400
	int init_display( Buffer_Param_T buffParam, int displayID );//默认4分屏，0左上，1右上，2左下，3右下，默认帧数据宽640高400
	int init_display( Buffer_Param_T buffParam, Display_Param_T displayParam );//自定义设置
	//发送显示数据
	int send_displaydata( Display_Data_T displaydata );
	//去初始化
	int reinit_display();
	
//private://后续可以修改为私有成员变量，通过函数进行参数的读写
	char m_caDispName[64];//显示器名称，主要用来进行显示器区分，否则容易出现问题
	int m_errcode;//显示类的错误码，0说明当前无错误
	int m_init;//初始化标志
	void* m_ctxptr;//控制参数
	Display_Param_T m_displayParam;//显示的相关参数
	Buffer_Param_T m_buffParam;//缓冲区相关参数
	void* m_buffptr;//缓冲区指针
	int m_buffridx;//缓冲区读索引
	int m_buffwidx;//缓冲区写索引
	int m_src_dmafd;//显示前dma文件
};

#endif