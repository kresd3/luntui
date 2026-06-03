#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

extern uint8 Gray_filter[MT9V03X_H][MT9V03X_W];
extern uint8 Gray_zip[MT9V03X_H/2][MT9V03X_W/2];
extern uint8 image_sobel[MT9V03X_H/2][MT9V03X_W/2];
//extern uint8 thresholds;

extern uint8 thresholds;


void zip(void);//Í¼ÏñÑ¹Ëõ
void sobel(void);
void fileOverview();//¸ßË¹ÂË²¨
uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);
void erzhihua();



#endif /* CODE_IMAGE_H_ */
