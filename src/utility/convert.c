/**
 * @file convert.c
 * @brief this file contain a functions for data converting
 * (c)2018 Sarov
 */
#include "convert.h"
#include "string.h"
 
/**
 * @brief copies float to byte array
 * @param f - float number or variable
 * @param array - byte array
 */
void FloatToByteArray(float f, uint8_t* array)
{
    uint32_t *ptr = (uint32_t*)&f;
    
    *(array+0) = *ptr>>24;
    *(array+1) = *ptr>>16;
    *(array+2) = *ptr>>8;
    *(array+3) = *ptr>>0;
}

/**
 * @brief copies byte array to float
 * @param array - byte array
 * @param pos - position in the array
 * @retval float representation
 */
float ByteArrayToFloat(uint8_t* array, uint16_t pos)
{
    uint32_t tmp;
    float* fp = (float*)&tmp;
    
    tmp =  *(array+pos+0); tmp <<= 8;
    tmp |= *(array+pos+1); tmp <<= 8;
    tmp |= *(array+pos+2); tmp <<= 8;
    tmp |= *(array+pos+3);
    return *fp;
}

/**
 *
 */
void memcpyRevers(uint8_t* dst, uint8_t* src, uint32_t size)
{
    while(size--) {
        *dst++ = *(src+size);
    }
}
//eof
