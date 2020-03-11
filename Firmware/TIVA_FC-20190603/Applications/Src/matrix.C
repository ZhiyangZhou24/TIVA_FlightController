#include "matrix.h"

//���󷽷�
//����m  
void matrix_set_m(struct _Matrix *m,int mm)
{
	m->m = mm; 
}

//����n  
void matrix_set_n(struct _Matrix *m,int nn)
{
	m->n = nn;
}

//��ʼ��  
void matrix_init(struct _Matrix *mm)
{
	mm->arr = (float *)malloc(mm->m * mm->n * sizeof(float));
}

//�ͷ�  
void matrix_free(struct _Matrix *mm)
{
	free(mm->arr);
}

//��ȡi,j���������  
//ʧ�ܷ���-31415,�ɹ�����ֵ  
float matrix_read(struct _Matrix *mm,int i,int j)
{
	
	if (i >= mm->m || j >= mm->n)  
    {
        return -31415;  
    }
		
    return *(mm->arr + i * mm->n + j);
}

//д��i,j���������  
//ʧ�ܷ���-1,�ɹ�����1  
int matrix_write(struct _Matrix *mm,int i,int j,float val)
{
	if (i >= mm->m || j >= mm->n)  
    {  
        return -1;  
    }  
      
    *(mm->arr + i * mm->n + j) = val;  
    return 1;  
}

//��������
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_add(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C)
{
	int i = 0;  
    int j = 0;  
      
    //�ж��Ƿ��������  
	if (A->m != B->m || A->n != B->n || \
        A->m != C->m || A->n != C->n)  
    {  
        return -1;  
    }  
    //����  
    for (i = 0;i < C->m;i++)  
    {  
        for (j = 0;j < C->n;j++)  
        {  
            matrix_write(C,i,j,matrix_read(A,i,j) + matrix_read(B,i,j));  
        }  
    }  
      
    return 1; 
}

//C = A - B  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_subtract(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C)
{
	int i = 0;  
    int j = 0;  
      
    //�ж��Ƿ��������  
    if (A->m != B->m || A->n != B->n || \
        A->m != C->m || A->n != C->n)  
    {  
        return -1;  
    }  
    //����  
    for (i = 0;i < C->m;i++)  
    {  
        for (j = 0;j < C->n;j++)  
        {  
            matrix_write(C,i,j,matrix_read(A,i,j) - matrix_read(B,i,j));  
        }  
    }  
      
    return 1; 
}

//C = A * B  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_multiply(struct _Matrix *A,struct _Matrix *B,struct _Matrix *C)
{
	int i = 0;  
    int j = 0;  
    int k = 0;  
    float temp = 0;  
      
    //�ж��Ƿ��������  
    if (A->m != C->m || B->n != C->n || \
        A->n != B->m)  
    {  
        return -1;  
    }  
    //����  
    for (i = 0;i < C->m;i++)  
    {  
        for (j = 0;j < C->n;j++)  
        {  
            temp = 0;  
            for (k = 0;k < A->n;k++)  
            {  
                temp += matrix_read(A,i,k) * matrix_read(B,k,j);  
            }  
            matrix_write(C,i,j,temp);  
        }
    }  
      
    return 1; 
}

//����ʽ��ֵ,ֻ�ܼ���2 * 2,3 * 3  
//ʧ�ܷ���-31415,�ɹ�����ֵ  
float matrix_det(struct _Matrix *A)
{
	float value = 0;  
      
    //�ж��Ƿ��������  
    if (A->m != A->n || (A->m != 2 && A->m != 3))  
    {  
        return -31415;  
    }  
    //����  
    if (A->m == 2)  
    {  
        value = matrix_read(A,0,0) * matrix_read(A,1,1) - matrix_read(A,0,1) * matrix_read(A,1,0);  
    }  
    else  
    {  
        value = matrix_read(A,0,0) * matrix_read(A,1,1) * matrix_read(A,2,2) + \
                matrix_read(A,0,1) * matrix_read(A,1,2) * matrix_read(A,2,0) + \
                matrix_read(A,0,2) * matrix_read(A,1,0) * matrix_read(A,2,1) - \
                matrix_read(A,0,0) * matrix_read(A,1,2) * matrix_read(A,2,1) - \
                matrix_read(A,0,1) * matrix_read(A,1,0) * matrix_read(A,2,2) - \
                matrix_read(A,0,2) * matrix_read(A,1,1) * matrix_read(A,2,0);  
    }  
      
    return value; 
}

//��ת�þ���,B = AT  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_transpos(struct _Matrix *A,struct _Matrix *B)
{
	int i = 0;  
    int j = 0;  
      
    //�ж��Ƿ��������  
    if (A->m != B->n || A->n != B->m)  
    {  
        return -1;  
    }  
    //����  
    for (i = 0;i < B->m;i++)  
    {  
        for (j = 0;j < B->n;j++)  
        {  
            matrix_write(B,i,j,matrix_read(A,j,i));  
        }  
    }  
      
    return 1;  
}

//�������,B = A^(-1)  
//�ɹ�����1,ʧ�ܷ���-1  
int matrix_inverse(struct _Matrix *A,struct _Matrix *B)
{
	  int i = 0;  
    int j = 0;  
    int k = 0;  
    struct _Matrix m;  
    float temp = 0;  
    float b = 0;  
      
    //�ж��Ƿ��������  
    if (A->m != A->n || B->m != B->n || A->m != B->m)  
    {  
        return -1;  
    }  
      
    /* 
    //�����2ά����3ά������ʽ�ж��Ƿ���� 
    if (A->m == 2 || A->m == 3) 
    {
        if (det(A) == 0) 
        { 
            return -1; 
        } 
    } 
    */  
      
    //�������m = A | B��ʼ��   
		matrix_set_m(&m,A->m);
		matrix_set_n(&m,2 * A->m);
		matrix_init(&m);
    for (i = 0;i < m.m;i++)  
    {
        for (j = 0;j < m.n;j++)  
        {
            if (j <= A->n - 1)  
            {  
                matrix_write(&m,i,j,matrix_read(A,i,j));  
            }  
            else  
            {  
                if (i == j - A->n)  
                {  
                    matrix_write(&m,i,j,1);  
                }  
                else  
                {  
                    matrix_write(&m,i,j,0);  
                }  
            }  
        }  
    }  
      
    //��˹��Ԫ  
    //�任������  
    for (k = 0;k < m.m - 1;k++)  
    {  
        //�������Ϊk,k����Ϊ0,���б任  
        if (matrix_read(&m,k,k) == 0)  
        {  
            for (i = k + 1;i < m.m;i++)  
            {  
                if (matrix_read(&m,i,k) != 0)  
                {  
                    break;  
                }  
            }  
            if (i >= m.m)  
            {  
                return -1;  
            }  
            else  
            {  
                //������  
                for (j = 0;j < m.n;j++)  
                {  
                    temp = matrix_read(&m,k,j);  
                    matrix_write(&m,k,j,matrix_read(&m,k + 1,j));  
                    matrix_write(&m,k + 1,j,temp);  
                }  
            }  
        }  
          
        //��Ԫ  
        for (i = k + 1;i < m.m;i++)  
        {  
            //��ñ���  
            b = matrix_read(&m,i,k) / matrix_read(&m,k,k);  
            //�б任  
            for (j = 0;j < m.n;j++)  
            {  
                temp = matrix_read(&m,i,j) - b * matrix_read(&m,k,j);  
                matrix_write(&m,i,j,temp);  
            }  
        }  
    }  
    //�任������  
    for (k = m.m - 1;k > 0;k--)  
    {  
        //�������Ϊk,k����Ϊ0,���б任  
        if (matrix_read(&m,k,k) == 0)  
        {  
            for (i = k + 1;i < m.m;i++)  
            {  
                if (matrix_read(&m,i,k) != 0)  
                {  
                    break;  
                }  
            }  
            if (i >= m.m)  
            {  
                return -1;  
            }  
            else  
            {  
                //������  
                for (j = 0;j < m.n;j++)  
                {  
                    temp = matrix_read(&m,k,j);  
                    matrix_write(&m,k,j,matrix_read(&m,k + 1,j));  
                    matrix_write(&m,k + 1,j,temp);  
                }  
            }  
        }  
          
        //��Ԫ  
        for (i = k - 1;i >= 0;i--)  
        {  
            //��ñ���  
            b = matrix_read(&m,i,k) / matrix_read(&m,k,k);  
            //�б任  
            for (j = 0;j < m.n;j++)  
            {  
                temp = matrix_read(&m,i,j) - b * matrix_read(&m,k,j);  
                matrix_write(&m,i,j,temp);  
            }  
        }  
    }  
    //����߷���Ϊ��λ����  
    for (i = 0;i < m.m;i++)  
    {  
        if (matrix_read(&m,i,i) != 1)  
        {  
            //��ñ���  
            b = 1 / matrix_read(&m,i,i);  
            //�б任  
            for (j = 0;j < m.n;j++)  
            {  
                temp = matrix_read(&m,i,j) * b;  
                matrix_write(&m,i,j,temp);  
            }  
        }  
    }  
    //��������  
    for (i = 0;i < B->m;i++)  
    {  
        for (j = 0;j < B->m;j++)  
        {  
            matrix_write(B,i,j,matrix_read(&m,i,j + m.m));  
        }  
    }  
    //�ͷ��������  
    matrix_free(&m);  
      
    return 1; 
}
