/*
 * Quaternions.h
 *
 *  Created on: 2 août 2022
 *      Author: EXtrA
 */

#ifndef INC_QUATERNIONS_H_
#define INC_QUATERNIONS_H_

#include "math.h"
typedef enum
{
    NONE,
    MADGWICK,
    MAHONY,
}QuatFilterSel;



void UpdateQuaternion (float A[],float G[] , float M[],float Q[]);

void NoFilter (float A[],float G[] , float M[],float Q[]);



void Mdgwick();


void Mahony();


#endif /* INC_QUATERNIONS_H_ */
