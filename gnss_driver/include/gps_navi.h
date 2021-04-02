/*
*
*  Integrated navigation Analysis 
*  AUTHOR: Zhao Ziwen
*  2020-07
*/
#ifndef GPS_NAVI_H
#define GPS_NAVI_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <cstdio>
using namespace std;

//单选：使用的是哪种语句
#define USING_NAVI
//GPGGA sentence
//$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77\r
class CNAVI
{
public:
	string InsWorkStatus;		//<1> 组合导航工作状态   enum格式
	string PosStatus;			//<2> 当前定位工作状态	 enum格式

	float latitude;				//<3> 纬度
	float longitude;			//<4> 经度
	float elevation;			//<5> 高程
    float splitNumber;

float qifu;
	float northspeed;			//<6> 北向速度	
	float eastspeed;			//<7> 东向速度
	float skyspeed;				//<8> 天向速度

	float rollAngle;			//<9> 翻滚（）	
	float pitchAngle;			//<10> 俯仰角（-90-90°）
	float yawAngle ;			//<11> 航向角（0-360°）
	
 float beiyong1;
	float beiyong2;
	
//int    bccVal; //校验
};

//处理heading语句的类
class CCopeNAVI
{
public:
	int CNAVIData( char* );
	CCopeNAVI();
	~CCopeNAVI();

public:
	CNAVI *p_heading;
	int num_new;  //当前处理完，产生了几条新语句
	
private:
	void SpliceData(char*);
	bool BccCheck( string& str );
	void ExtractData( string& str );

private:
	string raw_data;
	string a_frame;
};


CCopeNAVI::CCopeNAVI()
{
	raw_data = "";
	a_frame = "";
	p_heading = NULL;
#ifdef USING_NAVI
	p_heading = new CNAVI[14];
#endif
	num_new = 0;
}
CCopeNAVI::~CCopeNAVI()
{
	if(p_heading) delete []p_heading;
}

//拼接字符串
void CCopeNAVI::SpliceData(char* str_in) {
	raw_data += str_in;
	if(raw_data.size() > 4096)
	{
		raw_data = raw_data.substr(raw_data.size()-4096);
	}
}

bool CCopeNAVI::BccCheck( string& str ){//"$...."
	//printf("bccCheck.\n");
	if (str.empty())
		return false;
	
	int a = str[1], i=2;
	while(str[i] != '*')
	{
		a ^= str[i];
		++i;
	}
	
	int    bccVal; //校验
	stringstream ss;
	ss << str[i+1] << str[i+2];
	ss >> hex >> bccVal;//hex
	ss.clear();
	if ( bccVal == a )
		return true;
	else
		return false;
}
//提取数据
void CCopeNAVI::ExtractData( string& str )
{
	//protect the program, no more than 10 for once time!
	if(num_new > 10) return;
	
	//将字符串的分隔符由逗号变为空格（空内容则添加逗号），便于处理
	string str_copy;
	for (unsigned int i=0; i<str.size(); ++i)
	{
		//if(str[i] == ',' || str[i] == '*' || str[i] == ';')
		if(str[i] == ',' || str[i] == '*')
		{
				str_copy.push_back(' '); //更改为用“空格”做分隔符
				if(i>0 && str[i-1] == ',') //如果此项是空，用逗号代替实际内容
				{
					str_copy.push_back(','); 
					str_copy.push_back(' '); 
				}
		}
		else	
			str_copy.push_back(str[i]);
	}
	
#ifdef USING_NAVI
	//===============================================
	/*stringstream ss(str_copy);
	string tmp;
	
	//head：$HEADING3A
	for(int i = 0;i<11 ;i++)
		ss>>tmp;
	//ss >> tmp;*/
	int n = str_copy.find(';');
	string str_tmp = str_copy.substr(n);
	stringstream ss(str_tmp);
	
	string InsWorkStatus;		//<1> 组合导航工作状态   enum格式
	string PosStatus;			//<2> 当前定位工作状态	 enum格式

	float latitude;				//<3> 纬度
	float longitude;			//<4> 经度
	float elevation;			//<5> 高程
	float splitNumber;
        float qifu;

	float northspeed;			//<6> 北向速度	
	float eastspeed;			//<7> 东向速度
	float skyspeed;				//<8> 天向速度

	float rollAngle;			//<9> 翻滚（）	
	float pitchAngle;			//<10> 俯仰角（-90-90°）
	float yawAngle ;			//<11> 航向角（0-360°）
	 float beiyong1;
	float beiyong2;
	
	//<1> 组合导航工作状态   enum格式
	ss >> p_heading[num_new].InsWorkStatus;
	//<2> 当前定位工作状态	 enum格式
	ss >> p_heading[num_new].PosStatus;


	//<3> 纬度
	ss >> p_heading[num_new].latitude;
	//<4> 经度
	ss >> p_heading[num_new].longitude;
	//<5> 高程
	ss >> p_heading[num_new].elevation;
      
	ss >> p_heading[num_new].splitNumber;

	//<6> 北向速度
	ss >> p_heading[num_new].northspeed;
	//<7> 东向速度
	ss >> p_heading[num_new].eastspeed;
	//<8> 天向速度
	ss >> p_heading[num_new].skyspeed;


	//<9> 翻滚（）
	ss >> p_heading[num_new].rollAngle;
	//<10> 俯仰角（
	ss >> p_heading[num_new].pitchAngle;
	//<11> 航向角（0
	ss >> p_heading[num_new].yawAngle;


ss >> p_heading[num_new].beiyong1;
ss >> p_heading[num_new].beiyong2;


	//===============================================
#endif
	//num + 1
	num_new++;
}



int CCopeNAVI::CNAVIData( char* str_in )
{
	//set 0
	num_new = 0;
	//Splice with the old sentence
	SpliceData(str_in);  
	if(raw_data.size() < 20)
		return 0;
	
	//iteratively find a whole NEMA sentence, and analysis
	int pos_start = 0, pos_end = 0;
	while(pos_start!=string::npos && pos_end!=string::npos)
	{
		// find start
#ifdef USING_NAVI
		pos_start = raw_data.find(";", pos_end);//#HEADINGA
#endif
		if (pos_start == string::npos)
		{
			raw_data = raw_data.substr(pos_end+1);
			continue;
		}
		
		//find end
		pos_end = raw_data.find("\r", pos_start);
		if (pos_end == string::npos)
		{
			raw_data = raw_data.substr(pos_start);
			continue;
		}
		
		//get the whole sentence, and check
		a_frame.clear();
		a_frame = raw_data.substr(pos_start, pos_end-pos_start+1);
		/*if(!BccCheck(a_frame))  //check if ok!
		{
			//std::cout<<"check false!!!!!"<<std::endl;
			continue;
		}*/
		
		//analysis
		ExtractData(a_frame);
	}

	return num_new;
}


#endif


