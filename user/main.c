#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include "i2c.h"
#include "clock.h"
#include "math.h"

#define BUG_DETECT_PRINT(a,has_bug,no_bug) { if(a) \
printf("%s",has_bug); \
else \
printf("%s",no_bug);}

#define DEFAULT_MPU_HZ  (100)
#define ACCEL_WINDOW_H	400
#define ACCEL_WINDOW_L	-400
/*函数功能：根据匿名最新上位机协议写的显示姿态的程序
 *具体原理看匿名的讲解视频
 */
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void Data_Send_Status(float Pitch,float Roll,float Yaw,int16_t *gyro,int16_t *accel)
{
    unsigned char i=0;
    unsigned char _cnt=0,sum = 0;
    unsigned int _temp;
    u8 data_to_send[50];

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;

    _temp = (int)(Roll*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0-(int)(Pitch*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(Yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;
    //和校验
    for(i=0; i<_cnt; i++)
        sum+= data_to_send[i];
    data_to_send[_cnt++]=sum;

    //串口发送数据
    for(i=0; i<_cnt; i++)
        printf("%c",data_to_send[i]);
}

void Send_Data(int16_t *Gyro,int16_t *Accel)
{
    unsigned char i=0;
    unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
    u8 data_to_send[50];

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;


    data_to_send[_cnt++]=BYTE1(Accel[0]);
    data_to_send[_cnt++]=BYTE0(Accel[0]);
    data_to_send[_cnt++]=BYTE1(Accel[1]);
    data_to_send[_cnt++]=BYTE0(Accel[1]);
    data_to_send[_cnt++]=BYTE1(Accel[2]);
    data_to_send[_cnt++]=BYTE0(Accel[2]);

    data_to_send[_cnt++]=BYTE1(Gyro[0]);
    data_to_send[_cnt++]=BYTE0(Gyro[0]);
    data_to_send[_cnt++]=BYTE1(Gyro[1]);
    data_to_send[_cnt++]=BYTE0(Gyro[1]);
    data_to_send[_cnt++]=BYTE1(Gyro[2]);
    data_to_send[_cnt++]=BYTE0(Gyro[2]);
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=0;

    data_to_send[3] = _cnt-4;
    //和校验
    for(i=0; i<_cnt; i++)
        sum+= data_to_send[i];
    data_to_send[_cnt++]=sum;

    //串口发送数据
    for(i=0; i<_cnt; i++)
        printf("%c",data_to_send[i]);
}

/* a[0]航向角 a[1]俯仰角 a[2]横滚角 */
void evaluateDirectionCosine(int16_t accel[3],float a[3],float accel_res[3])
{
    float cos_a,cos_b,cos_c,sin_a,sin_b,sin_c;

    cos_a =	cos(a[0]);
    cos_b = cos(a[1]);
    cos_c = cos(a[2]);
    sin_a = sin(a[0]);
    sin_b = sin(a[1]);
    sin_c = sin(a[2]);

    accel_res[0] = (cos_c*cos_a-cos_b*sin_a*sin_c)*accel[0] + (cos_c*sin_a+cos_b*cos_a*sin_c)*accel[1] + ( sin_c*cos_b )*accel[2];
    accel_res[1] = ( -sin_c*cos_a-cos_b*sin_a*cos_c )*accel[0] + ( -sin_c*sin_a + cos_b*cos_a*cos_c )*accel[1] + ( cos_c*sin_b )*accel[2];
    accel_res[2] = ( sin_b*sin_c )*accel[0] + ( -sin_b*cos_c )*accel[1] + cos_b*accel[2];

}


/*修改后的基于四元数转换矩阵*/
void evaluateQuat(float accel_res[3],float accel[3],float q[4])
{
    accel_res[0]    = (q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])	*accel[0] + (2*q[0]*q[3]+2*q[1]*q[2])                           *accel[1]	+(2*q[1]*q[3]-2*q[0]*q[2])                          *accel[2];
    accel_res[1]	= (2*q[1]*q[2]-2*q[0]*q[3])						    *accel[0] + (q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3])	    *accel[1]   + (2*q[0]*q[1]+2*q[2]*q[3])                         *accel[2];
    accel_res[2]	=	(2*q[0]*q[2]+2*q[1]*q[3])						*accel[0] + (-2*q[0]*q[1]+2*q[2]*q[3])                          *accel[1]	+(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])	*accel[2];
}
/* 加速度值从基于载体坐标系转为参考坐标系 */
void acc_convert(long int accel_res[3],long int accel[3],float q[4])
{
    accel_res[0]    = (q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])	*accel[0] + (2*q[1]*q[2]-2*q[0]*q[3])	                        *accel[1]	+(2*q[0]*q[2]+2*q[1]*q[3])                          *accel[2];
    accel_res[1]	= (2*q[0]*q[3]+2*q[1]*q[2])					        *accel[0] + (q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3])	    *accel[1]   +(-2*q[0]*q[1]+2*q[2]*q[3])                         *accel[2];
    accel_res[2]	= (2*q[1]*q[3]-2*q[0]*q[2])							*accel[0] + (2*q[0]*q[1]+2*q[2]*q[3])                           *accel[1]	+(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])	*accel[2];
}

/* Function: 加速度一重积分  V(n) = V(n-1) + 0.5 * (a(n) + a(n-1)) * dt
*/
void acc_to_vel(float vel[3],float a[3],float ap[3],unsigned long time)
{
    int i;
    
    for(i=0;i<3;i++)
    {
        vel[i] += 0.5 * (a[i]+ap[i]) * time * 0.001;
    }
    for(i=0;i<3;i++)
    {
        ap[i] = a[i];
    }
}

void acc_to_disp(float disp[3],float a[3],float ap[3],float time)
{
    int i;
    float a_ave;

    for(i=0;i<3;i++)
    {
        a_ave = 0.5 * (a[i]+ap[i]);
        disp[i] += 0.5 * a_ave * time *time;
    }
    for(i=0;i<3;i++)
    {
        ap[i] = a[i];
    }
}

#define ACC_FILTER_COUNT	16
int16_t	acc_xyz_data[3][ACC_FILTER_COUNT] = {0};
int16_t	acc_data_index = 0;

/* 对原始数据加速度值进行滤波 */
void acc_filter(short int accel[3],long int acc_ave[3])
{
	int i,j;
	int32_t	acc_data_sum[3] = {0};
	
	//先进行一次机械窗口滤波
	for(i = 0; i < 3; i++)
	{
		if(accel[i] < ACCEL_WINDOW_H && accel[i] > ACCEL_WINDOW_L)
			accel[i] = 0;
	}
	
	//将i轴的加速度保存在acc_data_index列中
	for(i=0;i<3;i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}
	
	//acc_data_index循环加1
	acc_data_index++;
	
	if(acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
	
	//行求和
	for(i=0;i<3;i++)
	{
		for(j=0;j<ACC_FILTER_COUNT;j++)
		{
			acc_data_sum[i] +=  acc_xyz_data[i][j];
		}
		acc_ave[i] = acc_data_sum[i]/ACC_FILTER_COUNT;
	}
	
	//再对acc_ave进行一次机械窗口滤波
	for(i=0;i<3;i++)
	{
		if(acc_ave[i] < ACCEL_WINDOW_H && acc_ave[i] > ACCEL_WINDOW_L)
			acc_ave[i] = 0;
	}
}

/* 机械滤波 */
void movement_end_check(long int accel_n[3],long int vel[2][3])
{
	static unsigned int countx = 0, county = 0, countz = 0;
	//处理X轴
	if (accel_n[0]==0) //we count the number of acceleration samples that equals cero
	{ 
		countx++;
	}
	else 
	{ 
		countx =0;
	}
	if (countx>=25) //if this number exceeds 25, we can assume that velocity is cero
	{
		vel[1][0]=0;
		vel[0][0]=0;
	}
	//处理Y轴
	if (accel_n[1]==0) //we do the same for the Y axis
	{ 
		county++;
	}
	else 
	{ 
		county =0;
	}
	if (county>=25)
	{
		vel[1][1]=0;
		vel[0][1]=0;
	}
	//处理Z轴
	if(accel_n[2] == 0)
	{
		countz++;
	}
	if(countz >= 25)
	{
		vel[1][2]=0;
		vel[0][2]=0;
	}
}

void position(long int accel_n[2][3],long int vel[2][3],long int displayment[2][3])
{
	int 					i;

	for(i = 0; i < 3; i++)
	{
		vel[1][i] = vel[0][i] + ((accel_n[0][i]+accel_n[1][i])>>1);
		displayment[1][i] = displayment[0][i] + ((vel[1][i]+vel[0][i])>>1);
	}
	
	for(i = 0; i < 3; i++)
	{
		vel[0][i] = vel[1][i];
		displayment[0][i] = displayment[1][i];
	}
}

int main(void)
{
    struct int_param_s int_param;
    signed char gyro_orientation[9] = {1, 0, 0, 0,1, 0, 0, 0, 1};
    float  accel_g[3],accel_p[3] = {0},accel_final[3];
    float q[4],Pitch, Roll,Yaw;

    int16_t gyro[3], accel[3];
		long int vel[2][3]={0},disp[2][3] = {0},accel_ave[3],accel_res[2][3]={0};
    unsigned long timestamp,time_pre;
    short sensors = INV_XYZ_GYRO| INV_XYZ_ACCEL | INV_WXYZ_QUAT;
    unsigned char more;
    long quat[4];

		unsigned long time1,time2;

    /* debug用的变量 */
    int16_t accel_show[3],i,disp_show[3],vel_show[3];

    clock_conf();

    /* USART1 config 115200 8-N-1 */
    USART1_Config();
    printf("\r\n 这是一个MD移植程序 \r\n");

    ANBT_I2C_Configuration();		//IIC初始化
    //BUG_DETECT_PRINT(i2c_CheckDevice(0x68<<1),"\r\n 未检测到MPU6050 \r\n","\r\n 检测到MPU6050 \r\n");

    //BUG_DETECT_PRINT((result = DMP_MPU6050_DEV_CFG()),"\r\n MPU6050失败\r\n","\r\n MPU6050 \r\n");

    BUG_DETECT_PRINT(mpu_init(&int_param),"\r\n MPU6050初始化失败\r\n","\r\n MPU6050初始化成功\r\n");

    BUG_DETECT_PRINT(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL),"\r\n DMP设置传感器失败\r\n","\r\n DMP设置传感器成功\r\n");

    /* Push both gyro and accel data into the FIFO. */
    BUG_DETECT_PRINT(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL),"\r\n 设置FIFO失败\r\n","\r\n 设置FIFO成功\r\n");

    BUG_DETECT_PRINT(mpu_set_sample_rate(DEFAULT_MPU_HZ),"\r\n 设置采样率失败\r\n","\r\n 设置采样率成功\r\n");

    BUG_DETECT_PRINT(dmp_load_motion_driver_firmware(),"\r\n 加载固件失败\r\n","\r\n 加载成功\r\n");

    BUG_DETECT_PRINT(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)),"\r\n DMP设置初始方向失败\r\n","\r\n DMP设置初始方向成功\r\n");

    BUG_DETECT_PRINT(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT  | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL),"\r\n DMP初始化特性失败\r\n","\r\n DMP初始化特性成功\r\n");

    BUG_DETECT_PRINT(dmp_set_fifo_rate(DEFAULT_MPU_HZ),"\r\n 设置FIFO输出速率失败\r\n","\r\n 设置FIFO输出速率成功\r\n");

    run_self_test();

    mpu_set_dmp_state(1);
    
		delay_ms(10000);
		
    /* 用于除去重力，参考坐标系的Z轴加速度g转换成载体坐标系上的加速度 */
    accel_g[0] = 0;
    accel_g[1] = 0;
    accel_g[2] = 0.978833;
		
    while(1)
    {
				get_clock_ms(&time1);
        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
//        printf("%ld\n",timestamp);
        if((sensors & INV_WXYZ_QUAT))
        {
            /* DMP所得的四元数 */
            q[0]=quat[0] / 1073741824.0f;
            q[1]=quat[1] / 1073741824.0f;
            q[2]=quat[2] / 1073741824.0f;
            q[3]=quat[3] / 1073741824.0f;

            /* 由四元数所得的欧拉角，单位度 */
            Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]) *57.3; // pitch
            Roll = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)*57.3; // roll
            Yaw = 	atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])*57.3 ;		//感觉没有价值，注掉     
//			printf("%3f,%3f,%3f\n",Pitch,Roll, Yaw);
				}
				if((sensors & INV_XYZ_ACCEL))
				{
            /* accel_bias静止时的加速度值 long accel_bias[3]={700,-239,14890}
            accel[0] -= accel_bias[0];
            accel[1] -= accel_bias[1];
            accel[2] -= accel_bias[2];
            */

            /*
            原始数据的加速度值，除以16384得到m/s2实际加速度值，未除去重力
            printf("%3f,%3f,%3f\n",accel[0]/16384.0,accel[1]/16384.0, accel[2]/16384.0);
            */

            /*
            保留，由欧拉角得到转换矩阵，不准，不建议使用
            evaluateDirectionCosine(accel_g,gyro_res,accel_res);
            */

            /* 基于四元数的转换矩阵，将参考坐标系的重力加速度转换成载体坐标系的加速度 */
//            evaluateQuat(accel_res,accel_g,q);
					
						/* 先对加速度进行滤波 */
						acc_filter(accel,accel_ave);
						
            /* 将基于载体坐标系的加速度值转换为参考坐标系 */
            acc_convert(accel_res[1],accel_ave,q);
						
						for(i=0;i<3;i++)
						{
							if(accel_res[1][i] < ACCEL_WINDOW_H && accel_res[1][i] > ACCEL_WINDOW_L)
								accel_res[1][i] = 0;
						}
						
						accel_res[1][2] -= 14890;					
						
            accel_show [0] = accel_res[1][0]/16384;  
            accel_show [1] = accel_res[1][1]/16384;  
            accel_show [2] = accel_res[1][2]/16384;  
//            Send_Data(gyro,accel_show);
//						if(accel_res[1][1] || accel_res[0][1])
//							printf("%d,%d\r\n",accel_res[1][1],accel_res[0][1]);
						position(accel_res,vel,disp);
						
						disp_show[0] = disp[1][0]/16384;
						disp_show[1] = disp[1][1]/16384;
						disp_show[2] = disp[1][2]/16384;
						
						vel_show[0] = vel[1][0];
						vel_show[1] = vel[1][1];
						vel_show[2] = vel[1][2];
						
						Send_Data(vel_show,accel_show);
						movement_end_check(accel_res[1],vel);
						accel_res[0][0] = accel_res[1][0];
						accel_res[0][1] = accel_res[1][1];
						accel_res[0][2] = accel_res[1][2];

            /* 减去转换后的重力加速度，将载体坐标系各轴加速度增大至100倍，进行显示 */
//            accel_show[0] = (accel[0]/16384.0-accel_res[0])*100;
//            accel_show[1] = (accel[1]/16384.0-accel_res[1])*100;
//            accel_show[2] = (accel[2]/16384.0-accel_res[2])*100;
//            Send_Data(gyro,accel_show);

            /* 加速度一重积分得到速度 */
//            acc_to_vel(vel,accel_final,accel_p,timestamp - time_pre); 
//            time_pre = timestamp;
//            vel_show[0] = vel[0] *100 ;
//            vel_show[1] = vel[1] *100;
//            vel_show[2] = 0;
//            Send_Data(gyro,vel_show);
            
//            acc_to_disp(disp,accel_final,accel_p,(timestamp - time_pre)*0.001);
//            time_pre = timestamp;
//            disp_show[0] = disp[0] * 100;
//            disp_show[1] = disp[1] * 100;
//            disp_show[2] = disp[2] * 100;
//            Send_Data(gyro,disp_show);
//			Data_Send_Status(Pitch,Roll,-Yaw,gyro,accel);
//            Send_Data(gyro,(int16_t*)accel_show);
				//get_clock_ms(&time2);
				//printf("%ld\n",time2-time1);
				}
//			delay_ms(10);
    }
}


