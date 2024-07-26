/*=========================
 * ���е���ʽ�洢�ṹ�����ӣ�
 ==========================*/

#ifndef LINKQUEUE_H
#define LINKQUEUE_H

#include <stdio.h>
#include <stdlib.h>     
#include <headfile.h>
typedef int Status;

/* ״̬�� */
#define TRUE        1   // ��/��
#define FALSE       0   // ��/��
#define OK          1   // ͨ��/�ɹ�
#define ERROR       0   // ����/ʧ��

/* ����Ԫ�����Ͷ��� */



typedef void* QElemType;

// ����Ԫ�ؽṹ
typedef struct QNode {
    QElemType data;
    struct QNode* next;
} QNode, * QueuePtr;

// ���нṹ
typedef struct {
    QueuePtr front;     // ��ͷָ��
    QueuePtr rear;      // ��βָ��
} LinkQueue;            // ���е���ʽ�洢��ʾ


/*
 * ��ʼ��
 *
 * ����һ���յ����ӡ�
 * ��ʼ���ɹ��򷵻�OK�����򷵻�ERROR��
 *
 *��ע��
 * ����Ķ��д���ͷ���
 */
Status InitQueue(LinkQueue* Q);

/*
 * �п�
 *
 * �ж��������Ƿ������Ч���ݡ�
 *
 * ����ֵ��
 * TRUE : ����Ϊ��
 * FALSE: ���Ӳ�Ϊ��
 */
Status QueueEmpty(LinkQueue Q);

/*
 * ���
 *
 * ��Ԫ��e��ӵ�����β����
 */
Status EnQueue(LinkQueue* Q, QElemType e);

/*
 * ����
 *
 * �Ƴ�����ͷ����Ԫ�أ�����洢��e�С�
 */
Status DeQueue(LinkQueue* Q, QElemType* e);



#endif
