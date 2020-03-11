#ifndef _MATRIX_H  
#define _MATRIX_H

//ͷ�ļ�  
#include <stdio.h>  
#include <stdlib.h>  
  
//�������ݽṹ  
//��ά����  
struct _Matrix  
{
    int m;
    int n;
    float* arr;
};

//���󷽷�
//����m  
void matrix_set_m(struct _Matrix *m,int mm);  
//����n  
void matrix_set_n(struct _Matrix *m,int nn);  
//��ʼ��  
void matrix_init(struct _Matrix *m);  
//�ͷ�  
void matrix_free(struct _Matrix *m);  
//��ȡi,j���������  
//ʧ�ܷ���-31415,�ɹ�����ֵ  
float matrix_read(struct _Matrix *m,int i,int j);  
//д��i,j���������  
//ʧ�ܷ���-1,�ɹ�����1  
int matrix_write(struct _Matrix *m,int i,int j,float val); 

//��������
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_add(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C);  
//C = A - B  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_subtract(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C);  
//C = A * B  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_multiply(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C);  
//����ʽ��ֵ,ֻ�ܼ���2 * 2,3 * 3  
//ʧ�ܷ���-31415,�ɹ�����ֵ  
float matrix_det(struct _Matrix *A);  
//��ת�þ���,B = AT  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_transpos(struct _Matrix *A,struct _Matrix *B);  
//�������,B = A^(-1)  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_inverse(struct _Matrix *A,struct _Matrix *B);  
  
#endif  
