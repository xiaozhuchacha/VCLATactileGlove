#include "ros/ros.h"
#include "fsr_glove/glove.h"
#include "std_msgs/String.h"
#include "adc_acquire_ads1256.h"
#include <wiringPi.h>
#include <fstream>
#include <math.h>
#include <string.h>

#define NUM 26

//GPIO-MUX for fingers
const int addrS0=5;
const int addrS1=6;
const int addrS2=13;
const int addrS3=19;

//GPIO-MUX for palm
const int addrS00=12;
const int addrS01=16;
const int addrS10=20;
const int addrS11=21;

int addrMap[16][4]={{LOW,LOW,LOW,LOW},
		   {HIGH,LOW,LOW,LOW},
		   {LOW,HIGH,LOW,LOW},
		   {HIGH,HIGH,LOW,LOW},
		   {LOW,LOW,HIGH,LOW},
		   {HIGH,LOW,HIGH,LOW},
		   {LOW,HIGH,HIGH,LOW},
		   {HIGH,HIGH,HIGH,LOW},
		   {LOW,LOW,LOW,HIGH},
		   {HIGH,LOW,LOW,HIGH},
		   {LOW,HIGH,LOW,HIGH},
		   {HIGH,HIGH,LOW,HIGH},
		   {LOW,LOW,HIGH,HIGH},
		   {HIGH,LOW,HIGH,HIGH},
		   {LOW,HIGH,HIGH,HIGH},
		   {HIGH,HIGH,HIGH,HIGH}};

float bias[NUM]={
	1.1075, 1.1075, 1.1075, 0.9650, 1.5100, 1.2300, 1.2950, 1.5600, 1.2300, 1.2000,
	0.9624, 0.8556, 0.8556, 1.4423,
	1.1704, 0.8844, 0.8844, 1.3550,
	1.1704, 0.8844, 0.8844, 1.3550,
	1.5004, 1.3704, 1.1704, 1.1235
};

float coeff[NUM][2]={		//c1*exp(c2*(v-bias))
	{2.6179,3.6545},	//thumb proximal
	{2.6179,3.6740},	//thumb distal
	{2.6179,3.5560},	//index proximal
	{2.8000,3.9893},	//index distal
	{2.8570,4.9741},	//middle proximal
	{2.6179,4.2741},	//middle distal
	{2.6179,3.9741},	//ring proximal
	{2.6179,3.9741},	//ring distal
	{2.6179,4.5741},	//pinky proximal
	{2.6179,4.1741},	//pinky distal
	{17.0291,3.0775},	//palm(0,0)
	{15.5607,3.6000},	//palm(0,1)
	{15.5607,3.5000},       //palm(0,2)
	{38.4854,4.7553},	//palm(0,3)	
	{15.0423,3.7541},	//palm(1,0)
	{15.0737,2.8864},	//palm(1,1)
	{15.0737,2.9864},       //palm(1,2)
	{12.0448,5.0022},	//palm(1,3)
	{15.0423,3.4541},     	//palm(2,0)
	{15.0737,2.8864},       //palm(2,1)
	{15.0737,2.9864},       //palm(2,2)
	{10.0582,5.0022},       //palm(2,3)
	{15.0423,3.7841},     	//palm(3,0)
	{15.0423,3.8841},     	//palm(3,1)
	{15.0423,2.9864},     	//palm(3,2)
	{15.0423,5.0022},     	//palm(3,3)
};

float forces[NUM];

typedef enum mode {MODE_NORMAL, MODE_CALIB} force_mode;

void cb_forcectrl(const std_msgs::String::ConstPtr& msg)
{
	printf("\n OK %s",msg->data.c_str());
	for(int i=0;i<NUM;i++)
	{
		forces[i]=0;
	}
}

int main(int argc, char** argv)
{
	force_mode MODE;
	if(strcmp(argv[argc-1],"--normal")==0)
		MODE=MODE_NORMAL;
	else if(strcmp(argv[argc-1],"--calib")==0)
		MODE=MODE_CALIB;
	else
	{
		printf("args not correct\n");
		exit(0);
	}
	ros::init(argc,argv,"force_publish");
	ros::NodeHandle nh;

	ros::Publisher pub=nh.advertise<fsr_glove::glove>("force_msg",1000);	
	ros::Subscriber sub=nh.subscribe("tac_glove_imutracker_imuctrl",1000,cb_forcectrl);

	ADCAcquire adc_acquire_;

	printf("adc set!\n");
	//GPIO initialization after ADC board found

	wiringPiSetupGpio();
	pinMode(addrS0,OUTPUT);
	pinMode(addrS1,OUTPUT);
	pinMode(addrS2,OUTPUT);
	pinMode(addrS3,OUTPUT);
	pinMode(addrS00,OUTPUT);
	pinMode(addrS01,OUTPUT);
	pinMode(addrS10,OUTPUT);
	pinMode(addrS11,OUTPUT);

	//ros::Rate loop_rate(5);
	std::ofstream logfile("test.txt");
	while(ros::ok())
	{
		if(MODE==MODE_NORMAL)
		{
			int count=0;
			for(int i=0;i<10;i++)	//finger sensors
			{
				digitalWrite(addrS0,addrMap[i][0]);
				digitalWrite(addrS1,addrMap[i][1]);
				digitalWrite(addrS2,addrMap[i][2]);
				digitalWrite(addrS3,addrMap[i][3]);
				//delay(0.5);

				float v=adc_acquire_.read_channel(0);	//channel 0
				v-=bias[count];
				if(v>0.0)
					forces[count]=(float)coeff[count][0]*exp(coeff[count][1]*v);
				else
					forces[count]=0.0f;
				printf("finger %d: %f\n",i,forces[count]);
				count++;
			}
			for(int i=0;i<4;i++)	//palm sensor
			{
				digitalWrite(addrS00,addrMap[i][0]);
				digitalWrite(addrS01,addrMap[i][1]);
				for(int j=0;j<4;j++)
				{
					digitalWrite(addrS10,addrMap[j][0]);
					digitalWrite(addrS11,addrMap[j][1]);
					//delay(0.5);
				
					float v=adc_acquire_.read_channel(1);	//channel 1
					v-=bias[count];
					if(v>0.0)
            					forces[count]=(float)coeff[count][0]*exp(coeff[count][1]*v);
					else
						forces[count]=0.0f;
					printf("row=%d col=%d: %f\n",i,j,forces[count]);
					count++;
				}
			}
			
			//loop_rate.sleep();
			count=0;	
			fsr_glove::glove msg;
			forces[24]=forces[20];forces[25]=forces[21];
      			msg.header.stamp = ros::Time::now();
			for(int i=0;i<5;i++)	//other fingers
			{
				msg.segments[i].force={forces[count],forces[count+1],0.0,0.0};
				count+=2;
			}
			for(int i=5;i<9;i++)	//palm
			{
				msg.segments[i].force={forces[count],forces[count+1],forces[count+2],forces[count+3]};
        			count+=4;	
			}
			pub.publish(msg);
			//ros::spinOnce();
		}
		else if(MODE==MODE_CALIB)
		{
			//delay(100);
			float v=adc_acquire_.read_channel(0);   //channel 0
      			printf("voltage: %f\n",v);
			//logfile<<std::fixed<<v<<"\n";			
		}
	}
	logfile.close();
	return 0;
}
