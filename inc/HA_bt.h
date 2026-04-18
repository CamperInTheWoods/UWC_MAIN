

#ifndef HA_BT_H
#define HA_BT_H

#define BM20_P2_0_PORT   (2)      //PC8 //BM20_P2_0
#define BM20_P2_0_PIN    (8)

#define BM20_P2_4_PORT   (2)      //PC9  //BM20_P2_4
#define BM20_P2_4_PIN    (9)

#define BM20_MFB_PORT    (2)      //PC2  //BM20_MFB
#define BM20_MFB_PIN     (2)

#define BM20_RESET_PORT  (2)      //PC3  //BM20_RESET
#define BM20_RESET_PIN   (3)      //

#define BM20_EAN_PORT    (2)      //PC4  //BM20_EAN
#define BM20_EAN_PIN     (4)      //

void XMOS_Reset();
void BT_MakeCall();

#endif
