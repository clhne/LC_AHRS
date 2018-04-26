#include "key.h"
#define KEY PAin(15)
/**************************************************************************
�������ܣ�������ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void key_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ��PA�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); // �����趨������ʼ��GPIOA
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫��
**************************************************************************/
u8 double_click(u8 time) {
    static u8 flag_key, count_key, double_key;
    static u16 count_single, forever_count;
    if (KEY == 0) { //������־λδ��1
        forever_count++;
    } else {
        forever_count = 0;
    }
    if (KEY == 0 && flag_key == 0) {
        flag_key = 1;
    }
    if (count_key == 0) {
        if (flag_key == 1) {
            double_key++;
            count_key = 1;
        }
        if (double_key == 2) {
            double_key = 0;
            count_single = 0;
            return 2; // ˫��ִ�е�ָ��
        }
    }
    if (KEY == 1)	{
        flag_key = 0;
        count_key = 0;
    }
    if (double_key == 1) {
        count_single++;
        if (count_single > time && forever_count < time) {
            double_key = 0;
            count_single = 0;
            return 1; // ����ִ�е�ָ��
        }
        if(forever_count > time) {
            double_key = 0;
            count_single = 0;
        }
    }
    return 0;
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������
**************************************************************************/
u8 click(void) {
    static u8 flag_key = 1; // �������ɿ���־
    if (flag_key && KEY == 0) {
        flag_key = 0;
        return 1;	// ��������
    } else if (KEY == 1) {
        flag_key = 1;
    }
    return 0; // �ް�������
}

/**************************************************************************
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������2s
**************************************************************************/
u8 long_press(void) {
    static u16 long_press_count, long_press_flag;
    if (long_press_flag == 0 && KEY == 0) { // ������־λδ��1
        long_press_count++;
    } else {
        long_press_count = 0;
    }
    if (long_press_count > 200) {
        long_press_flag = 1;
        long_press_count = 0;
        return 1;
    }
    if (long_press_flag == 1) { // ������־λ��1
        long_press_flag = 0;
    }
    return 0;
}
