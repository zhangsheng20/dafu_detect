#include "data_send.h"


unsigned char data_send_buf[14];
extern Point2f SendYawPitchErrorCurrent;


extern int IsDetectDafuCenter;
extern Point2f ShootArmourPitchYawError;        //    需要打击的装甲板的中心坐标
extern Point2f DafuCenterPitchYawError;              //大符中心坐标
extern Point2f PredcitShootArmourCenter;
extern Serial sel;


//识别不到发30000
//大符01 yaw pitch
//自瞄34 yaw pitch
void SendDataToInfantry()
{
    // SendYawPitchErrorCurrent.x=-9.6;
    //SendYawPitchErrorCurrent.y=-8.6;


  char buff[8];
  //ShootArmourPitchYawError.x+=2.0;
  //ShootArmourPitchYawError.y+=-20.0;

  //Data_Code( (int)(-ShootArmourPitchYawError.x*100),  (int)(-ShootArmourPitchYawError.y*100));
  //Data_Code( (int)(-PredcitShootArmourCenterPitchYawError.x*100),  (int)(-PredcitShootArmourCenterPitchYawError.y*100));
  Data_Code( (int)((PredcitShootArmourCenter.x-320)*100),  (int)((PredcitShootArmourCenter.y-250)*100));
    for (int i = 0; i < 8; i++)
        buff[i] = data_send_buf[i];

    sel.writeData(buff, 8);

}


unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth)
{
  unsigned char byte_crc = 0;
  for (unsigned char i = 0; i < data_lenth; i++) {
    byte_crc += InputBytes[i];
  }
  return byte_crc;
}


void Data_disintegrate_u16(unsigned int Data, unsigned char *LData,  unsigned char *HData)
{
  *LData = Data & 0XFF;          // 0xFF = 1111 1111
  *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}

#define BYTE0(dwTemp) (*((char*)&(dwTemp)))
#define BYTE1(dwTemp) (*((char*)&(dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)&(dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)&(dwTemp)+3))

void Data_disintegrate_s16(int Data, unsigned char *LData,unsigned char *HData)
{
   *LData=BYTE0(Data);
   *HData=BYTE1(Data);
}



//识别不到发30000
//大符01 yaw pitch
//自瞄34 yaw pitch



void Data_Code( int x_Data,  int y_Data)
{
  int length = 8;
  data_send_buf[0] = 0xFF;
  data_send_buf[1] = length;
  data_send_buf[2] = 0x02;
  //
  Data_disintegrate_s16(x_Data, &data_send_buf[3], &data_send_buf[4]);
  Data_disintegrate_s16(y_Data, &data_send_buf[5], &data_send_buf[6]);


  //
  data_send_buf[length - 1] = Add_CRC(data_send_buf, length - 1);
}


