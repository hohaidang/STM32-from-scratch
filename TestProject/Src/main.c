
#include <stdio.h>
#include<stdlib.h>
#include<string.h>

int numbers[] = { 456,345,678,567,890,456,3456,123,765,456,896,456,678,987,000,145,90};

int  someData = 90;

void array_fill_numbers(int pNumbers[], unsigned int len)
{

    for ( unsigned int i = 0 ; i < len ; i++)
    {
        pNumbers[i] = rand() % 1000;
    }

}

void display_numbers(int *pNumbers, unsigned int len, char *pMessage)
{
    printf("%s",pMessage);

    for (unsigned i = 0 ; i < len ; i++)
    {
        printf("%5d  ",pNumbers[i]);
    }

    printf("\n");
}


void swap_numbers(int *x,int *y)
{
     int temp=*x;
     *x=*y;
     *y=temp;

//     void (*jump_addr) (void);
//     jump_addr = (void *)0x20000008;
//     jump_addr();

     someData = 10;
}


void bubble_sort(int *pNumbers , unsigned int len)
{

    int i,j,flag=0;

    for(i=0;i<len-1;i++)
    {
        flag=0;

        for(j=0;j<len-1-i;j++)
        {
            if(pNumbers[j] > pNumbers[j+1])
            {
                swap_numbers(&pNumbers[j],&pNumbers[j+1]);
                flag=1;
            }
        }

        if(flag==0)
            break;
    }
}

void insertion_sort(int *pNumbers , unsigned int len)
{

     int i,j,num;

     for(i=1 ; i<len ; i++)
     {
         j=i-1;

         num = pNumbers[i];

         while( (j>-1) && (pNumbers[j] > num) )
         {
             pNumbers[j+1] = pNumbers[j];
             j--;
         }

         pNumbers[j+1]=num;
     }


}



int main()
{

    unsigned int len = sizeof(numbers)/sizeof(int);

    array_fill_numbers(numbers,len);

    display_numbers(numbers,len,"B-unsorted array :");

    bubble_sort(numbers,len);

    display_numbers(numbers,len,"B-sorted array   :");

    array_fill_numbers(numbers,len);

    display_numbers(numbers,len,"I-unsorted array :");

    insertion_sort(numbers,len);

    display_numbers(numbers,len,"I-sorted array   :");



    return 0;
}

