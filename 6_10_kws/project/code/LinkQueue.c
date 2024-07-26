/*=========================
 * ���е���ʽ�洢�ṹ�����ӣ�
 ==========================*/



#include "LinkQueue.h"  

/*
 * ��ʼ��
 *
 * ����һ���յ����ӡ�
 * ��ʼ���ɹ��򷵻�OK�����򷵻�ERROR��
 *
 *��ע��
 * ����Ķ��д���ͷ���
 */
Status InitQueue(LinkQueue* Q) {
    if(Q == NULL) {
        return ERROR;
    }
    
    (*Q).front = (*Q).rear = (QueuePtr) malloc(sizeof(QNode));
    if(!(*Q).front) {
        return -2;
    }
    
    (*Q).front->next = NULL;
    
    return OK;
}

/*
 * �п�
 *
 * �ж��������Ƿ������Ч���ݡ�
 *
 * ����ֵ��
 * TRUE : ����Ϊ��
 * FALSE: ���Ӳ�Ϊ��
 */
Status QueueEmpty(LinkQueue Q) {
    if(Q.front == Q.rear) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/*
 * ���
 *
 * ��Ԫ��e��ӵ�����β����
 */
Status EnQueue(LinkQueue* Q, QElemType e) {
    QueuePtr p;
    
    if(Q == NULL || (*Q).front == NULL) {
        return ERROR;
    }
    
    p = (QueuePtr) malloc(sizeof(QNode));
    if(!p) {
        return -2;
    }
    
    p->data = e;
    p->next = NULL;
    
    (*Q).rear->next = p;
    (*Q).rear = p;
    
    return OK;
}

/*
 * ����
 *
 * �Ƴ�����ͷ����Ԫ�أ�����洢��e�С�
 */
Status DeQueue(LinkQueue* Q, QElemType* e) {
    QueuePtr p;
    
    if(Q == NULL || (*Q).front == NULL || (*Q).front == (*Q).rear) {
        return ERROR;
    }
    
    p = (*Q).front->next;
    *e = p->data;
    
    (*Q).front->next = p->next;
    if((*Q).rear == p) {
        (*Q).rear = (*Q).front;
    }
    
    free(p);
    
    return OK;
}


