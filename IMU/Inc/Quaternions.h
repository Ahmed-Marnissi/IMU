

#ifndef INC_QUATERNIONS_H_
#define INC_QUATERNIONS_H_

#include "math.h"
typedef enum
{
    NONE,
    MADGWICK,
    MAHONY,
}QuatFilterSel;

extern QuatFilterSel FilterSelecter;



extern size_t NFilterIteration ;

extern double DeltaT;
extern float Kp ;
extern float Ki ;

void UpdateQuaternion (float A[],float G[] , float M[],float Q[]);

void NoFilter (float A[],float G[] , float M[],float Q[]);



void Mdgwick();


void Mahony(float A[],float G[] , float M[],float Q[]);


#endif /* INC_QUATERNIONS_H_ */
