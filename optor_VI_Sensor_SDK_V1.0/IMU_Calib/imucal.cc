#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <signal.h> // signal functions
#include <pthread.h>
#include <fstream>

#define    IMU_FRAME_LEN              32
#define    COLOR_NONE                 "\033[0m"
#define    FONT_COLOR_RED             "\033[1;31m"
#define    FONT_COLOR_GREEN           "\033[1;32m"
#define    FONT_COLOR_YELLOW          "\033[1;33m"
#define    FONT_COLOR_BLUE            "\033[1;34m"

using   namespace   std;

bool shut_down_imu=false;
int  imu_fd = 0;

static int abs(int x)
{ 
    if(x<0) return -x;  
} 

static int visensor_open_port(const char* dev_str)
{
    int fd = open(dev_str, O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return(-1);
    }

    if(fcntl(fd, F_SETFL, 0)<0)
        printf("fcntl failed!\n");
    if(isatty(STDIN_FILENO)==0)
        printf("standard input is not a terminal device\n");
    return fd;
}

static int find_55aa(unsigned char* buf,int len)
{
    int i;
    for(i=0; i<len-1; i++)
    {
        if(buf[i]==0x55)
            if(buf[i+1]==0xAA)
                return i;
    }
    if(buf[len-1]==0x55)
        return -1;
    return -2;
}

static int visensor_get_imu_frame(int fd, unsigned char* imu_frame)
{
    unsigned char imu_frame_buf[2*IMU_FRAME_LEN];
    memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
    memset(imu_frame,0,IMU_FRAME_LEN);

    int num_get = 0;
    static int flush_count = 0;
    flush_count = (flush_count+1)%200;
    if(flush_count==0)
        tcflush(fd,TCIFLUSH);
    struct timeval getIMUTime;
    gettimeofday(&getIMUTime,NULL);
    double start_time = getIMUTime.tv_sec+0.000001*getIMUTime.tv_usec;
    while(!shut_down_imu)
    {
        //读到32个以上字节
        while(num_get<IMU_FRAME_LEN)
        {
            int temp_read_get = read(fd,&imu_frame_buf[num_get],IMU_FRAME_LEN);
            if(temp_read_get<=0)
            {
                gettimeofday(&getIMUTime,NULL);
                double current_time = getIMUTime.tv_sec+0.000001*getIMUTime.tv_usec;
                if(current_time - start_time < 0.1)
                    continue;//读取错误，不得往num_get上加
                else
                    return -1;
            }
            num_get+=temp_read_get;
        }//实际读到了num_get字节数

        //查询55AA的位置
        int position_55aa = find_55aa(imu_frame_buf,num_get);
        //如果查不到55AA(-2)，末尾也没有55，那么此次读取失败，清空imu_frame_buf,从头再来
        if(position_55aa == -2)
        {
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
            num_get = 0;
            continue;
        }
        //如果末尾是55，那么清空imu_frame_buf,然后将首位置为55
        if(position_55aa == -1)
        {
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
            num_get = 1;
            imu_frame_buf[0] = 0x55;
            continue;
        }
        //如果找到了55AA，那么首先检验有效帧长度
        int valid_len = num_get - position_55aa;
        //如果有效帧长度不足32，那么将数据移动到帧头，然后继续读取剩余帧
        if(valid_len<32)
        {
            for(int i=0; i<valid_len; i++)
                imu_frame_buf[i] = imu_frame_buf[position_55aa+i];
            for(int i=valid_len; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;
            num_get = valid_len;
            continue;
        }
        //如果有效帧长度达到32，那么检查校验和
        unsigned char checksum = 0;
        for(int i=2; i<31; i++)checksum += imu_frame_buf[position_55aa+i];
        //如果校验和不正确，那么清除该帧头和之前的所有数据，然后将剩余数据移动到帧头，再继续读取剩余数据
        if(checksum != imu_frame_buf[position_55aa+31])
        {
            for(int i=2; i<valid_len; i++)
                imu_frame_buf[i-2] = imu_frame_buf[position_55aa+i];
            for(int i=valid_len-2; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;
            num_get = valid_len-2;
            continue;
        }
        //如果校验和正确，而有效帧长度超过了32，那么说明已经有数据溢出，应立即清除串口缓存,然后重新开始读取
        if(valid_len>32)
        {
            tcflush(fd,TCIFLUSH);
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
            num_get = 0;
            continue;
        }

        //Timestamp algorithm
        static double time_interval=0.005;
        static double last_time=-1.0,last_pred=-1.0;
        double new_pred;

        double new_time = getIMUTime.tv_sec+0.000001*getIMUTime.tv_usec-0.0035;
    	static int get_imu_num = imu_frame_buf[2];
        if(last_time<0||last_pred<0)
        {
            last_time = last_pred = new_time;
            return -2;
        }
        //如果校验和正确，那么输出该帧数据
        memcpy(imu_frame,&imu_frame_buf[position_55aa],IMU_FRAME_LEN);
        break;
    }

    return 0;
}
static int visensor_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if ( tcgetattr( fd,&oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 32;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");	int imu_fd = 0;
        return -1;
    }
    //printf("set done!\n");
    return 0;
}
static int visensor_send_imu_frame(int fd, unsigned char* data, int len)
{
    return write(fd,data,len);
}
static int visensor_Start_IMU(char *port)
{
    // serial port
    int fd=visensor_open_port(port);
    if(fd<0)
    {
    	printf(FONT_COLOR_RED"无法打开！\n"COLOR_NONE);
        return -1;
    }
    printf(FONT_COLOR_GREEN"成功打开！\n"COLOR_NONE);

    if(visensor_set_opt(fd,115200,8,'N',1)<0)
    {
        printf("visensor_set_opt error...\r\n");
        return 0;
    }
    //printf("visensor_set_opt(fd,115200,8,'N',1) success...\r\n");

    static unsigned char sendframe[10]= {0x55,0xAA,0x02};
    sendframe[9]=sendframe[3]+sendframe[4]+sendframe[5]+sendframe[6]+sendframe[7]+sendframe[8];
    visensor_send_imu_frame(fd,sendframe,10);

    //Create imu_data thread
    imu_fd=fd;
}

void process_imu_data(void )
{
	int  get_accel_data_x_up[3] = {0}, get_accel_data_x_down[3] = {0};
	int  get_accel_data_y_up[3] = {0}, get_accel_data_y_down[3] = {0};
	int  get_accel_data_z_up[3] = {0}, get_accel_data_z_down[3] = {0};
	int  get_imu_x_up_data_cnt = 0, get_imu_x_down_data_cnt = 0;
	int  get_imu_y_up_data_cnt = 0, get_imu_y_down_data_cnt = 0;
	int  get_imu_z_up_data_cnt = 0, get_imu_z_down_data_cnt = 0;
	unsigned char imu_frame[IMU_FRAME_LEN] = {0};
	int get_char = 0;
	int get_x_offset = 0, get_y_offset = 0, get_z_offset = 0;
	int get_standard_g = 0;

	printf("请将摄像头的X轴正方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_x_up_data_cnt < 400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_x_up[0] += *(short*)(&imu_frame[9]);
			get_accel_data_x_up[1] += *(short*)(&imu_frame[11]);
			get_accel_data_x_up[2] += *(short*)(&imu_frame[13]);
			get_imu_x_up_data_cnt++;
			//printf("0:%d 1:%d 2:%d\n", get_accel_data_x_up[0], get_accel_data_x_up[1], get_accel_data_x_up[2]);
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_x_up_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_x_up_data_cnt);
	get_accel_data_x_up[0] /= get_imu_x_up_data_cnt;
	get_accel_data_x_up[1] /= get_imu_x_up_data_cnt;
	get_accel_data_x_up[2] /= get_imu_x_up_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_x_up[0], get_accel_data_x_up[1], get_accel_data_x_up[2]);

	printf("请将摄像头的X轴负方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_x_down_data_cnt<400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_x_down[0] += *(short*)(&imu_frame[9]);
			get_accel_data_x_down[1] += *(short*)(&imu_frame[11]);
			get_accel_data_x_down[2] += *(short*)(&imu_frame[13]);
			get_imu_x_down_data_cnt++;
			//printf("num:%d 0:%d 1:%d 2:%d\n",get_imu_x_down_data_cnt, tmp[0], tmp[1], tmp[2]);
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_x_down_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_x_down_data_cnt);
	get_accel_data_x_down[0] /= get_imu_x_down_data_cnt;
	get_accel_data_x_down[1] /= get_imu_x_down_data_cnt;
	get_accel_data_x_down[2] /= get_imu_x_down_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_x_down[0], get_accel_data_x_down[1], get_accel_data_x_down[2]);

	printf("请将摄像头的Y轴正方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_y_up_data_cnt < 400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_y_up[0] += *(short*)(&imu_frame[9]);
			get_accel_data_y_up[1] += *(short*)(&imu_frame[11]);
			get_accel_data_y_up[2] += *(short*)(&imu_frame[13]);
			get_imu_y_up_data_cnt++;
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_y_up_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_y_up_data_cnt);
	get_accel_data_y_up[0] /= get_imu_y_up_data_cnt;
	get_accel_data_y_up[1] /= get_imu_y_up_data_cnt;
	get_accel_data_y_up[2] /= get_imu_y_up_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_y_up[0], get_accel_data_y_up[1], get_accel_data_y_up[2]);

	printf("请将摄像头的Y轴负方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_y_down_data_cnt < 400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_y_down[0] += *(short*)(&imu_frame[9]);
			get_accel_data_y_down[1] += *(short*)(&imu_frame[11]);
			get_accel_data_y_down[2] += *(short*)(&imu_frame[13]);
			get_imu_y_down_data_cnt++;
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_y_down_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_y_down_data_cnt);
	get_accel_data_y_down[0] /= get_imu_y_down_data_cnt;
	get_accel_data_y_down[1] /= get_imu_y_down_data_cnt;
	get_accel_data_y_down[2] /= get_imu_y_down_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_y_down[0], get_accel_data_y_down[1], get_accel_data_y_down[2]);

	printf("请将摄像头的Z轴正方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_z_up_data_cnt < 400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_z_up[0] += *(short*)(&imu_frame[9]);
			get_accel_data_z_up[1] += *(short*)(&imu_frame[11]);
			get_accel_data_z_up[2] += *(short*)(&imu_frame[13]);
			get_imu_z_up_data_cnt++;
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_z_up_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_z_up_data_cnt);
	get_accel_data_z_up[0] /= get_imu_z_up_data_cnt;
	get_accel_data_z_up[1] /= get_imu_z_up_data_cnt;
	get_accel_data_z_up[2] /= get_imu_z_up_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_z_up[0], get_accel_data_z_up[1], get_accel_data_z_up[2]);

	printf("请将摄像头的Z轴负方向朝上放置，然后按回车键，静止两秒钟后继续...\n");
	while(get_char=getchar()!='\n');
	tcflush(imu_fd,TCIFLUSH);
	while(get_imu_z_down_data_cnt < 400)
	{
		if(visensor_get_imu_frame(imu_fd, imu_frame) == 0)
		{
			get_accel_data_z_down[0] += *(short*)(&imu_frame[9]);
			get_accel_data_z_down[1] += *(short*)(&imu_frame[11]);
			get_accel_data_z_down[2] += *(short*)(&imu_frame[13]);
			get_imu_z_down_data_cnt++;
			printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b请稍等...[%d]/[400]", get_imu_z_down_data_cnt);
		}
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b已完成...[%d]/[400]\n", get_imu_z_down_data_cnt);
	get_accel_data_z_down[0] /= get_imu_z_down_data_cnt;
	get_accel_data_z_down[1] /= get_imu_z_down_data_cnt;
	get_accel_data_z_down[2] /= get_imu_z_down_data_cnt;
	printf("平均加速度： %d, %d, %d\n", get_accel_data_z_down[0], get_accel_data_z_down[1], get_accel_data_z_down[2]);

	get_x_offset = (get_accel_data_x_up[0] + get_accel_data_x_down[0]) / 2;
	get_y_offset = (get_accel_data_y_up[1] + get_accel_data_y_down[1]) / 2;
	get_z_offset = (get_accel_data_z_up[2] + get_accel_data_z_down[2]) / 2;
	printf("\n加速度偏移值为： %d, %d, %d\n", get_x_offset, get_y_offset, get_z_offset);

	get_standard_g = (abs(get_accel_data_x_up[0] - get_accel_data_x_down[0]) +
			 abs(get_accel_data_y_up[1] - get_accel_data_y_down[1]) +
			 abs(get_accel_data_z_up[2] - get_accel_data_z_down[2])) / 6;
	printf("标准重力值为： %d\n", get_standard_g);

	printf("校准完成，正在写入文件...\n");
	FILE *fp;
	if(!(fp=fopen("imucaldata.txt","wb")))
	{
		puts("打开文件成败"); 
	}
	fprintf(fp,"%d, %d, %d, %d", get_x_offset, get_y_offset, get_z_offset, get_standard_g); 
	fclose(fp);	
	printf("已写入文件，按回车键退出校准程序\n");
	while(get_char=getchar()!='\n');
}

int main(int argc, char* argv[])
{
	printf("\n*************Optor IMU 加速度计校准程序***************\n");
	if(!argv[1])
	{
		printf("请键入USB串口设备名，例如：imucal /dev/ttyUSB0\n");
		printf("您可以使用ls /dev/ttyUSB* 命令查看所有USB串口设备\n");
		return -1;
	}
	printf("正在打开串口设备：");
	printf(FONT_COLOR_YELLOW"%s"COLOR_NONE, argv[1]);
	printf(" ...\n");

	visensor_Start_IMU(argv[1]);

	process_imu_data();

	return 0;
}
