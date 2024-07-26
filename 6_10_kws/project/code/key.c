#include "key.h"
#include "headfile.h"
volatile KEY_STATUS key_status;  
volatile KEY_FLAG key_flag; 
void init_key()
{//���뿪�س�ʼ��
    gpio_init(SW1,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    gpio_init(SW2,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    //������ʼ��   
    gpio_init(KEYU,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    gpio_init(KEYD,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    gpio_init(KEYL,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    gpio_init(KEYR,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    gpio_init(KEYEnter,GPI,0,SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
    KS.keyU_status=1;
    KS.keyD_status=1;
    KS.keyL_status=1;
    KS.keyR_status=1;
    KS.KEYEnter_status=1;
 
}
void key_scan()
{       //��ȡ���뿪��״̬
        KS.sw1_status = gpio_get_level(SW1);
        KS.sw2_status = gpio_get_level(SW2);

        //��TFT����ʾ���뿪��״̬
        //lcd_showstr(0,1,"SW1 STATUS:");     lcd_showint32(12*8,1,sw1_status,1);
        //lcd_showstr(0,2,"SW2 STATUS:");     lcd_showint32(12*8,2,sw2_status,1);
        
        //ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
        //���水��״̬
        KS.keyU_last_status = KS.keyU_status;
        KS.keyD_last_status = KS.keyD_status;
        KS.keyL_last_status = KS.keyL_status;
        KS.keyR_last_status = KS.keyR_status;
        KS.KEYEnter_last_status = KS.KEYEnter_status;
        //��ȡ��ǰ����״̬
        KS.keyU_status = gpio_get_level(KEYU);
        KS.keyD_status = gpio_get_level(KEYD);
        KS.keyL_status = gpio_get_level(KEYL);
        KS.keyR_status = gpio_get_level(KEYR);//��˵������Ҫ���ľ�һ��?
        KS.KEYEnter_status = gpio_get_level(KEYEnter);
        
        //��⵽��������֮��  ���ſ���λ��־λ
        if(KS.keyU_status && !KS.keyU_last_status)    KF.keyU_flag = 1;
        if(KS.keyD_status && !KS.keyD_last_status)    KF.keyD_flag = 1;
        if(KS.keyL_status && !KS.keyL_last_status)    KF.keyL_flag = 1;
        if(KS.keyR_status && !KS.keyR_last_status)    KF.keyR_flag = 1;
        if(KS.KEYEnter_status && !KS.KEYEnter_last_status)    KF.KEYEnter_flag = 1;
}     

